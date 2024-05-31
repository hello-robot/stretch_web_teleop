#!/usr/bin/env python3

# Standard Imports
import sys
import threading
from enum import Enum
from typing import Dict, List, Optional, Tuple

# Third-Party Imports
import message_filters
import numpy as np
import numpy.typing as npt
import pcl
import rclpy
import ros2_numpy
import tf2_py as tf2
import tf2_ros
import yaml
from geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped, Vector3
from inverse_jacobian_controller import (
    InverseJacobianController,
    TerminationCriteria,
    remaining_time,
)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from tf2_geometry_msgs import PointStamped, PoseStamped
from tf_transformations import quaternion_about_axis, quaternion_multiply

# Local Imports
from stretch_web_teleop.action import MoveToPregrasp


class MoveToPregraspState(Enum):
    """
    Move-to-pregrasp proceeds in the following sequential states:
      1. Moving/rotating the base to the target pose.
      2. Moving the arm & wrist to the target pose.

    TODO: Consider adding an initial state that stows the arm.
    """

    ROTATE_BASE = 1
    MOVE_ARM = 2
    MOVE_WRIST = 3
    # TODO: Instead, make a terminal state.

    def next(self) -> Tuple[bool, MoveToPregraspState]:
        """
        Get the next state.

        Returns
        -------
        Tuple[bool, MoveToPregraspState]: Whether the sequence is done, and the next state.
        """
        if self == MoveToPregraspState.ROTATE_BASE:
            return False, MoveToPregraspState.MOVE_ARM
        elif self == MoveToPregraspState.MOVE_ARM:
            return False, MoveToPregraspState.MOVE_WRIST
        elif self == MoveToPregraspState.MOVE_WRIST:
            return True, MoveToPregraspState.ROTATE_BASE


class MoveToPregraspNode(Node):
    """
    The MoveToPregrasp node exposes an action server that takes in the
    (x, y) pixel coordinates of an operator's click on the Realsense
    camera feed. It then moves the robot so its end-effector is
    aligned with the clicked pixel, making it easy for the user to grasp
    the object the pixel is a part of.
    """

    BASE_LINK = "base_link"
    END_EFFECTOR_LINK = "link_grasp_center"
    ODOM_FRAME = "odom"

    def __init__(
        self,
        image_params_file: str,
        tf_timeout_secs: float = 0.5,
        action_timeout_secs: float = 100.0,  # TODO: lower!
    ):
        """
        Initialize the MoveToPregraspNode

        Parameters
        ----------
        image_params_file: The path to the YAML file configuring the video streams
            that are sent to the web app. This is necessary because this node has to undo
            the transforms before deprojecting the clicked pixel.
        tf_timeout_secs: The timeout in seconds for TF lookups.
        action_timeout_secs: The timeout in seconds for the action server.
        """
        super().__init__("move_to_pregrasp")

        # Load the video stream parameters
        with open(image_params_file, "r") as params:
            self.image_params = yaml.safe_load(params)

        # Initialize TF2
        self.tf_timeout = Duration(seconds=tf_timeout_secs)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create the inverse jacobian controller to execute motions
        self.controller = InverseJacobianController(self, self.tf_buffer)

        # Subscribe to the Realsense's RGB, pointcloud, and camera info feeds
        self.latest_realsense_msgs_lock = threading.Lock()
        self.latest_realsense_msgs = None
        camera_rgb_subscriber = message_filters.Subscriber(
            self,
            Image,
            "/camera/color/image_raw",
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        point_cloud_subscriber = message_filters.Subscriber(
            self,
            PointCloud2,
            "/camera/depth/color/points",
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.camera_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [
                camera_rgb_subscriber,
                point_cloud_subscriber,
            ],
            queue_size=1,
            # TODO: Tune the max allowable delay between RGB and pointcloud messages
            slop=1.0,  # seconds
            allow_headerless=False,
        )
        self.camera_synchronizer.registerCallback(self.realsense_cb)
        self.p_lock = threading.Lock()
        self.p = None  # The camera's projection matrix
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.camera_info_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Create the action timeout
        self.action_timeout = Duration(seconds=action_timeout_secs)

    def initialize(self) -> bool:
        """
        Initialize the MoveToPregraspNode.

        This is necessary because ROS must be spinning while the controller
        is being initialized.

        Returns
        -------
        bool: Whether the initialization was successful.
        """
        # Initialize the controller
        ok = self.controller.initialize()
        if not ok:
            self.get_logger().error(
                "Failed to initialize the inverse jacobian controller"
            )
            return False

        # Create the shared resource to ensure that the action server rejects all
        # new goals while a goal is currently active.
        self.active_goal_request_lock = threading.Lock()
        self.active_goal_request = None

        # Create the action server
        self.action_server = ActionServer(
            self,
            MoveToPregrasp,
            "move_to_pregrasp",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def camera_info_cb(self, msg: CameraInfo) -> None:
        """
        Callback for the camera info subscriber. Save the camera's projection matrix.

        Parameters
        ----------
        msg: The camera info message.
        """
        with self.p_lock:
            self.p = np.array(msg.p).reshape(3, 4)

    def realsense_cb(self, rgb_msg: Image, pointcloud_msg: PointCloud2) -> None:
        """
        Callback for the Realsense camera feed subscriber. Save the latest RGB and pointcloud messages.

        Parameters
        ----------
        rgb_msg: The RGB image message.
        pointcloud_msg: The pointcloud message.
        """
        with self.latest_realsense_msgs_lock:
            self.latest_realsense_msgs = (rgb_msg, pointcloud_msg)

    def goal_callback(self, goal_request: MoveToPregrasp.Goal) -> GoalResponse:
        """
        Accept a goal if this action does not already have an active goal, else reject.

        Parameters
        ----------
        goal_request: The goal request message.
        """
        self.get_logger().info(f"Received request {goal_request}")

        # Reject the goal if no camera info has been received yet
        with self.p_lock:
            if self.p is None:
                self.get_logger().info(
                    "Rejecting goal request since no camera info has been received yet"
                )
                return GoalResponse.REJECT

        # Reject the goal if no Realsense messages have been received yet
        with self.latest_realsense_msgs_lock:
            if self.latest_realsense_msgs is None:
                self.get_logger().info(
                    "Rejecting goal request since no Realsense messages have been received yet"
                )
                return GoalResponse.REJECT

        # Reject the goal is there is already an active goal
        with self.active_goal_request_lock:
            if self.active_goal_request is not None:
                self.get_logger().info(
                    "Rejecting goal request since there is already an active one"
                )
                return GoalResponse.REJECT

        # Accept the goal
        self.get_logger().info("Accepting goal request")
        self.active_goal_request = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, _: ServerGoalHandle) -> CancelResponse:
        """
        Always accept client requests to cancel the active goal.

        Parameters
        ----------
        goal_handle: The goal handle.
        """
        self.get_logger().info("Received cancel request, accepting")
        return CancelResponse.ACCEPT

    async def execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> MoveToPregrasp.Result:
        """
        Execute the goal, by rotating the robot's base and adjusting the arm lift/length
        to align the end-effector with the clicked pixel.

        Parameters
        ----------
        goal_handle: The goal handle.

        Returns
        -------
        MoveToPregrasp.Result: The result message.
        """
        self.get_logger().info(f"Got request {goal_handle.request}")
        start_time = self.get_clock().now()

        # Undo any transformation that were applied to the raw camera image before sending it
        # to the web app
        raw_u, raw_v = goal_handle.request.u, goal_handle.request.v
        if (
            "realsense" in self.image_params
            and "default" in self.image_params["realsense"]
        ):
            u, v = self.inverse_transform_pixel(
                raw_u, raw_v, self.image_params["realsense"]["default"]
            )
        else:
            u, v = raw_u, raw_v

        # Get the latest Realsense messages
        with self.latest_realsense_msgs_lock:
            rgb_msg, pointcloud_msg = self.latest_realsense_msgs
        pointcloud = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(
            pointcloud_msg
        )  # N x 3 array

        # Deproject the clicked pixel to get the 3D coordinates of the clicked point
        x, y, z = self.deproject_pixel_to_point(u, v, pointcloud)
        self.get_logger().info(
            f"Closest point to clicked pixel (camera frame): {(x, y, z)}"
        )

        # Determine how the robot should orient its gripper to align with the clicked pixel
        if (
            goal_handle.request.pregrasp_direction
            == MoveToPregrasp.Goal.PREGRASP_DIRECTION_HORIZONTAL
        ):
            horizontal_grasp = True
        elif (
            goal_handle.request.pregrasp_direction
            == MoveToPregrasp.Goal.PREGRASP_DIRECTION_VERTICAL
        ):
            self.get_logger().warn(
                "Vertical grasp not implemented yet. Defaulting to horizontal grasp."
            )
            horizontal_grasp = False
        else:  # auto
            self.get_logger().warn(
                "Auto grasp not implemented yet. Defaulting to horizontal grasp."
            )
            horizontal_grasp = True

        # Get the goal end effector pose
        ok, goal_pose = self.get_goal_pose(
            x, y, z, pointcloud_msg.header, horizontal_grasp
        )
        if not ok:
            self.get_logger().error("Failed to get goal pose")
            goal_handle.abort()
            return MoveToPregrasp.Result()
        self.get_logger().info(f"Goal Pose: {goal_pose}")

        state = MoveToPregraspState.MOVE_WRIST  # TODO: change!
        future = None
        while rclpy.ok():
            self.get_logger().info(f"State: {state}", throttle_duration_sec=1.0)
            # Check if a cancel has been requested
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                goal_handle.canceled()
            # Check if the action has timed out
            if (self.get_clock().now() - start_time) > self.action_timeout:
                self.get_logger().error("Goal timed out")
                goal_handle.abort()
                return MoveToPregrasp.Result()

            # Move the robot
            if future is None:
                if state == MoveToPregraspState.ROTATE_BASE:
                    articulated_joints = [
                        InverseJacobianController.JOINT_BASE_REVOLUTION
                    ]
                    additional_joint_positions = {}
                elif state == MoveToPregraspState.MOVE_WRIST:
                    articulated_joints = [
                        InverseJacobianController.JOINT_WRIST_YAW,
                        InverseJacobianController.JOINT_WRIST_PITCH,
                        InverseJacobianController.JOINT_WRIST_ROLL,
                    ]
                    additional_joint_positions = {
                        # We only have to command one finger since they are controlled by the same motor
                        "joint_gripper_finger_left": 0.165,
                    }
                elif state == MoveToPregraspState.MOVE_ARM:
                    if horizontal_grasp:
                        articulated_joints = [
                            InverseJacobianController.JOINT_ARM_LIFT,
                        ]
                    else:
                        articulated_joints = [
                            InverseJacobianController.JOINT_ARM_L3,
                            InverseJacobianController.JOINT_ARM_L2,
                            InverseJacobianController.JOINT_ARM_L1,
                            InverseJacobianController.JOINT_ARM_L0,
                        ]
                    additional_joint_positions = {}
                future = self.executor.create_task(
                    self.controller.move_to_end_effector_pose(
                        goal=goal_pose,
                        articulated_joints=articulated_joints,
                        termination=TerminationCriteria.ZERO_VEL,
                        additional_joint_positions=additional_joint_positions,
                        timeout_secs=remaining_time(
                            self.get_clock().now(),
                            start_time,
                            self.action_timeout,
                            return_secs=True,
                        ),
                    )
                )
            # Check if the robot is done moving
            elif future.done():
                try:
                    ok = future.result()
                    if not ok:
                        raise Exception("Failed to move to goal pose")
                except Exception as e:
                    self.get_logger().error(f"{e}")
                    goal_handle.abort()
                    return MoveToPregrasp.Result()
                done, state = state.next()
                if done:
                    break
                else:
                    future = None

            # TODO: Send feedback!

        goal_handle.succeed()
        result = MoveToPregrasp.Result()
        self.active_goal_request = None
        self.get_logger().info("Goal succeeded!")
        return result

    def inverse_transform_pixel(self, u: int, v: int, params: Dict) -> Tuple[int, int]:
        """
        Undo the transformations applied to the raw camera image before
        sending it to the web app. This is necessary so the pixel coordinates
        of the click correspond to the camera's intrinsic parameters.

        TODO: Eventually the forward transforms (in configure_video_streams.py) and
        inverse transforms (here) should be combined into one helper file, to
        ensure consistent interpretation of the parameters.

        Parameters
        ----------
        u: The horizontal coordinate of the clicked pixel in the web app.
        v: The vertical coordinate of the clicked pixel in the web app.
        params: The transformation parameters.

        Returns
        -------
        Tuple[int, int]: The transformed pixel coordinates.
        """
        if "crop" in params and params["crop"] is not None:
            if "x_min" in params["crop"]:
                u += params["crop"]["x_min"]
            if "y_min" in params["crop"]:
                v += params["crop"]["y_min"]

        if "rotate" in params and params["rotate"] is not None:
            # Get the image dimensions. Note that (w, h) are dimensions of the raw image,
            # while (x, y) are coordinates on the transformed image.
            with self.latest_realsense_msgs_lock:
                # An image is guaranteed to exist by the precondition of accepting the goal
                img_msg = self.latest_realsense_msgs[0]
            w, h = img_msg.width, img_msg.height

            if params["rotate"] == "ROTATE_90_CLOCKWISE":
                u, v = v, h - u
            elif params["rotate"] == "ROTATE_180":
                u, v = w - u, h - v
            elif params["rotate"] == "ROTATE_90_COUNTERCLOCKWISE":
                u, v = w - v, u
            else:
                raise ValueError(
                    "Invalid rotate image value: options are ROTATE_90_CLOCKWISE, ROTATE_180, or ROTATE_90_COUNTERCLOCKWISE"
                )

        return u, v

    def deproject_pixel_to_point(
        self, u: int, v: int, pointcloud: npt.NDArray[np.float32]
    ) -> Tuple[float, float, float]:
        """
        Deproject the clicked pixel to get the 3D coordinates of the clicked point.

        Parameters
        ----------
        u: The horizontal coordinate of the clicked pixel.
        v: The vertical coordinate of the clicked pixel.
        pointcloud: The pointcloud array of size (N, 3).

        Returns
        -------
        Tuple[float, float, float]: The 3D coordinates of the clicked point.
        """
        # Get the ray from the camera origin to the clicked point
        ray_dir = np.linalg.pinv(self.p)[:3, :] @ np.array([u, v, 1])
        ray_dir /= np.linalg.norm(ray_dir)

        # Find the point that is closest to the ray
        p, r = pointcloud, ray_dir
        closest_point_idx = np.argmin(
            np.linalg.norm(
                p - np.multiply((p @ r).reshape((-1, 1)), r.reshape((1, 3))), axis=1
            )
        )

        return p[closest_point_idx]

    def get_goal_pose(
        self,
        x: float,
        y: float,
        z: float,
        header: Header,
        horizontal_grasp: bool,
    ) -> Tuple[bool, PoseStamped]:
        """
        Get the goal end effector pose.

        Parameters
        ----------
        x: The x-coordinate of the clicked point.
        y: The y-coordinate of the clicked point.
        z: The z-coordinate of the clicked point.
        header: The header of the pointcloud message.
        horizontal_grasp: Whether the goal pose should be horizontal.

        Returns
        -------
        ok: Whether the goal pose was successfully calculated.
        PoseStamped: The goal end effector pose.
        """
        stb = tf2_ros.StaticTransformBroadcaster(self)

        # Get the goal position in camera frame
        goal_position = PointStamped()
        goal_position.header = header
        goal_position.point = Point(x=x, y=y, z=z)

        # Get the goal orientation in base frame
        try:
            goal_position_base = self.tf_buffer.transform(
                goal_position, self.BASE_LINK, timeout=self.tf_timeout
            )
            if horizontal_grasp:
                # The goal orientation is (0,0,0,1) in base link frame, rotated
                # so +x points towards the clicked point
                theta = np.arctan2(
                    goal_position_base.point.y, goal_position_base.point.x
                )
                rot = quaternion_about_axis(theta, [0, 0, 1])
            else:
                # The goal orientation is rotated -90deg around z and +90deg around y
                # in base link frame
                rot = quaternion_about_axis(np.pi / 2, [0, 1, 0])
                rot = quaternion_multiply(
                    quaternion_about_axis(-np.pi / 2, [0, 0, 1]), rot
                )
            # We have to treat the orientation as a pose since tf2_geometry_msgs.py
            # doesn't register QuaternionStamped
            goal_orientation = PoseStamped()
            goal_orientation.header = header
            goal_orientation.header.frame_id = self.BASE_LINK
            goal_orientation.pose.orientation = Quaternion(
                x=rot[0], y=rot[1], z=rot[2], w=rot[3]
            )
        except (
            tf2.ConnectivityException,
            tf2.ExtrapolationException,
            tf2.InvalidArgumentException,
            tf2.LookupException,
            tf2.TimeoutException,
            tf2.TransformException,
        ) as error:
            self.get_logger().error(
                f"Failed to transform goal pose to the odom frame: {error}"
            )
            return False, PoseStamped()

        # Convert the goal pose to the odom frame so it stays fixed even as the robot moves
        try:
            goal_position_odom = self.tf_buffer.transform(
                goal_position_base, self.ODOM_FRAME, timeout=self.tf_timeout
            )
            goal_orientation_odom = self.tf_buffer.transform(
                goal_orientation, self.ODOM_FRAME, timeout=self.tf_timeout
            )
            goal_pose_odom = PoseStamped()
            goal_pose_odom.header = goal_position_odom.header
            goal_pose_odom.pose.position = goal_position_odom.point
            goal_pose_odom.pose.orientation = goal_orientation_odom.pose.orientation
        except (
            tf2.ConnectivityException,
            tf2.ExtrapolationException,
            tf2.InvalidArgumentException,
            tf2.LookupException,
            tf2.TimeoutException,
            tf2.TransformException,
        ) as error:
            self.get_logger().error(
                f"Failed to transform goal pose to the odom frame: {error}"
            )
            return False, PoseStamped()

        stb.sendTransform(
            TransformStamped(
                header=goal_pose_odom.header,
                child_frame_id="goal",
                transform=Transform(
                    translation=Vector3(
                        x=goal_pose_odom.pose.position.x,
                        y=goal_pose_odom.pose.position.y,
                        z=goal_pose_odom.pose.position.z,
                    ),
                    rotation=goal_pose_odom.pose.orientation,
                ),
            )
        )

        return True, goal_pose_odom


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    image_params_file = args[0]

    move_to_pregrasp = MoveToPregraspNode(image_params_file)
    move_to_pregrasp.get_logger().info("Created!")

    # Use a MultiThreadedExecutor so that subscriptions, actions, etc. can be
    # processed in parallel.
    executor = MultiThreadedExecutor(num_threads=4)

    # Spin in the background, as the node initializes
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(move_to_pregrasp,),
        kwargs={"executor": executor},
        daemon=True,
    )
    spin_thread.start()

    # Initialize the node
    move_to_pregrasp.initialize()

    # Spin in the foreground
    spin_thread.join()

    move_to_pregrasp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
