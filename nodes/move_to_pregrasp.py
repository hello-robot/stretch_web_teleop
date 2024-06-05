#!/usr/bin/env python3

# Standard Imports
from __future__ import annotations  # Required for type hinting MoveToPregraspState.next

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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from stretch_ik_control import StretchIKControl, TerminationCriteria, remaining_time
from tf2_geometry_msgs import PointStamped, PoseStamped
from tf_transformations import quaternion_about_axis, quaternion_multiply

# Local Imports
from stretch_web_teleop.action import MoveToPregrasp


class MoveToPregraspState(Enum):
    """
    Move-to-pregrasp proceeds in the following sequential states:
      1. Opens the wrist and moves it to either the horizontal or vertical pre-grasp position.
      2. Rotates the robot's base to align the end-effector with the clicked pixel.
      3. Adjusts the arm's lift/length to align the end-effector with the clicked pixel.
    """

    # TODO: Stow wrist first!

    MOVE_WRIST = 0
    ROTATE_BASE = 1
    MOVE_ARM = 2
    TERMINAL = 3

    def next(self) -> MoveToPregraspState:
        """
        Get the next state.

        Returns
        -------
        MoveToPregraspState: The next state.
        """
        if self != MoveToPregraspState.TERMINAL:
            return MoveToPregraspState(self.value + 1)


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

    DISTANCE_TO_OBJECT = 0.1  # meters

    def __init__(
        self,
        image_params_file: str,
        tf_timeout_secs: float = 0.5,
        action_timeout_secs: float = 15.0,
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
        self.controller = StretchIKControl(self, self.tf_buffer)

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

        # Start the timer
        start_time = self.get_clock().now()

        # Initialize the feedback
        feedback = MoveToPregrasp.Feedback()
        feedback.initial_distance_m = -1.0

        def distance_callback(err: npt.NDArray[np.float32]) -> None:
            self.get_logger().error(f"Distance error: {err}")
            distance = np.linalg.norm(err[:3])
            if feedback.initial_distance_m < 0.0:
                feedback.initial_distance_m = distance
            else:
                feedback.remaining_distance_m = distance

        # Undo any transformation that were applied to the raw camera image before sending it
        # to the web app
        raw_scaled_u, raw_scaled_v = (
            goal_handle.request.scaled_u,
            goal_handle.request.scaled_v,
        )
        if (
            "realsense" in self.image_params
            and "default" in self.image_params["realsense"]
        ):
            params = self.image_params["realsense"]["default"]
        else:
            params = None
        u, v = self.inverse_transform_pixel(raw_scaled_u, raw_scaled_v, params)
        self.get_logger().info(
            f"Clicked pixel after inverse transform (camera frame): {(u, v)}"
        )

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
            horizontal_grasp = False
        else:  # auto
            self.get_logger().warn(
                "Auto grasp not implemented yet. Defaulting to horizontal grasp."
            )
            horizontal_grasp = True

        # Get the goal end effector pose
        ok, goal_pose = self.get_goal_pose(
            x, y, z, pointcloud_msg.header, horizontal_grasp, publish_tf=True
        )
        if not ok:
            self.get_logger().error("Failed to get goal pose")
            goal_handle.abort()
            return MoveToPregrasp.Result()
        self.get_logger().info(f"Goal Pose: {goal_pose}")
        # goal_handle.succeed()
        # self.active_goal_request = None
        # return MoveToPregrasp.Result()

        state = MoveToPregraspState(0)
        future = None
        terminate_future = False
        result = MoveToPregrasp.Result()
        rate = self.create_rate(10.0)
        while rclpy.ok():
            self.get_logger().info(f"State: {state}", throttle_duration_sec=1.0)
            # Check if a cancel has been requested
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                result.status = MoveToPregrasp.Result.STATUS_CANCELLED
                goal_handle.canceled()
                break
            # Check if the action has timed out
            if (self.get_clock().now() - start_time) > self.action_timeout:
                self.get_logger().error("Goal timed out")
                result.status = MoveToPregrasp.Result.STATUS_TIMEOUT
                goal_handle.abort()
                break

            # Move the robot
            if future is None:
                articulated_joints = []
                additional_joint_positions = {}
                if state == MoveToPregraspState.MOVE_WRIST:
                    additional_joint_positions[StretchIKControl.JOINT_GRIPPER] = 0.84
                    if horizontal_grasp:
                        additional_joint_positions.update(
                            {
                                StretchIKControl.JOINT_WRIST_YAW: 0.0,
                                StretchIKControl.JOINT_WRIST_PITCH: 0.0,
                                StretchIKControl.JOINT_WRIST_ROLL: 0.0,
                            }
                        )
                    else:
                        additional_joint_positions.update(
                            {
                                StretchIKControl.JOINT_WRIST_YAW: 0.0,
                                StretchIKControl.JOINT_WRIST_PITCH: -np.pi / 2.0,
                                StretchIKControl.JOINT_WRIST_ROLL: 0.0,
                            }
                        )
                elif state == MoveToPregraspState.ROTATE_BASE:
                    articulated_joints += [StretchIKControl.JOINT_BASE_REVOLUTION]
                elif state == MoveToPregraspState.MOVE_ARM:
                    if horizontal_grasp:
                        articulated_joints += [
                            StretchIKControl.JOINT_ARM_LIFT,
                        ]
                    else:
                        articulated_joints += [
                            *StretchIKControl.JOINTS_ARM,
                        ]
                if len(articulated_joints) > 0:
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
                            check_cancel=lambda: terminate_future,
                            err_callback=distance_callback,
                        )
                    )
                else:
                    future = self.executor.create_task(
                        self.controller.move_to_joint_positions(
                            joint_positions=additional_joint_positions,
                            timeout_secs=remaining_time(
                                self.get_clock().now(),
                                start_time,
                                self.action_timeout,
                                return_secs=True,
                            ),
                            check_cancel=lambda: terminate_future,
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
                    result.status = MoveToPregrasp.Result.STATUS_FAILURE
                    goal_handle.abort()
                    break
                state = state.next()
                if state == MoveToPregraspState.TERMINAL:
                    self.get_logger().info("Goal succeeded")
                    result.status = MoveToPregrasp.Result.STATUS_SUCCESS
                    goal_handle.succeed()
                    break
                else:
                    future = None

            # Send feedback. If we are not controlling any joints with the inverse
            # Jacobian, then we need to calculate the error here.
            if len(articulated_joints) == 0:
                ok, err = self.controller.get_err(
                    goal_pose,
                    timeout=remaining_time(
                        self.get_clock().now(),
                        start_time,
                        self.action_timeout,
                    ),
                )
                if ok:
                    distance_callback(err)
            feedback.elapsed_time = (self.get_clock().now() - start_time).to_msg()
            goal_handle.publish_feedback(feedback)

            # Sleep
            rate.sleep()

        # Perform cleanup
        self.active_goal_request = None
        if future is not None:
            terminate_future = True
            future.cancel()
        return result

    def inverse_transform_pixel(
        self, scaled_u: int, scaled_v: int, params: Optional[Dict]
    ) -> Tuple[int, int]:
        """
        First, unscale the u and v coordinates. Then, undo the transformations
        applied to the raw camera image before sending it to the web app.
        This is necessary so the pixel coordinates of the click correspond to
        the camera's intrinsic parameters.

        TODO: Eventually the forward transforms (in configure_video_streams.py) and
        inverse transforms (here) should be combined into one helper file, to
        ensure consistent interpretation of the parameters.

        Parameters
        ----------
        scaled_u: The horizontal coordinate of the clicked pixel in the web app, in [0.0, 1.0].
        scaled_v: The vertical coordinate of the clicked pixel in the web app, in [0.0, 1.0].
        params: The transformation parameters.

        Returns
        -------
        Tuple[int, int]: The transformed pixel coordinates.
        """
        # Get the image dimensions. Note that (w, h) are dimensions of the raw image,
        # while (u, v) are coordinates on the transformed image.
        with self.latest_realsense_msgs_lock:
            # An image is guaranteed to exist by the precondition of accepting the goal
            img_msg = self.latest_realsense_msgs[0]
        w, h = img_msg.width, img_msg.height

        # First, unscale the u and v coordinates
        if (
            "rotate" in params
            and params["rotate"] is not None
            and "ROTATE_90" in params["rotate"]
        ):
            u = scaled_u * h
            v = scaled_v * w
        else:
            u = scaled_u * w
            v = scaled_v * h

        # Then, undo the crop
        if "crop" in params and params["crop"] is not None:
            if "x_min" in params["crop"]:
                u += params["crop"]["x_min"]
            if "y_min" in params["crop"]:
                v += params["crop"]["y_min"]

        # Then, undo the rotate
        if "rotate" in params and params["rotate"] is not None:
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
        publish_tf: bool = False,
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
        publish_tf: Whether to publish the goal pose as a TF frame.

        Returns
        -------
        ok: Whether the goal pose was successfully calculated.
        PoseStamped: The goal end effector pose.
        """
        if publish_tf:
            static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Get the goal position in camera frame
        goal_pose = PoseStamped()
        goal_pose.header = header
        goal_pose.pose.position = Point(x=x, y=y, z=z)
        goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Convert to base frame
        try:
            goal_pose_base = self.tf_buffer.transform(
                goal_pose, self.BASE_LINK, timeout=self.tf_timeout
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

        # The goal orientation is (0,0,0,1) in base link frame, rotated
        # so +x points towards the clicked point
        theta = np.arctan2(
            goal_pose_base.pose.position.y, goal_pose_base.pose.position.x
        )
        rot = quaternion_about_axis(theta, [0, 0, 1])
        if not horizontal_grasp:
            # For vertical grasp, the goal orientation is also rotated +90deg around y
            rot = quaternion_multiply(rot, quaternion_about_axis(np.pi / 2, [0, 1, 0]))
        goal_pose_base.pose.orientation = Quaternion(
            x=rot[0], y=rot[1], z=rot[2], w=rot[3]
        )

        # Adjust the goal position by the distance to the object
        if horizontal_grasp:
            xy = np.array(
                [goal_pose_base.pose.position.x, goal_pose_base.pose.position.y]
            )
            xy_dist = np.linalg.norm(xy)
            if xy_dist < self.DISTANCE_TO_OBJECT:
                self.get_logger().error(
                    f"Clicked point is too close to the robot: {xy_dist} < {self.DISTANCE_TO_OBJECT}"
                )
                return False, PoseStamped()
            xy = xy / xy_dist * (xy_dist - self.DISTANCE_TO_OBJECT)
            goal_pose_base.pose.position.x, goal_pose_base.pose.position.y = xy
        else:
            goal_pose_base.pose.position.z += self.DISTANCE_TO_OBJECT

        # Convert the goal pose to the odom frame so it stays fixed even as the robot moves
        try:
            goal_pose_odom = self.tf_buffer.transform(
                goal_pose_base, self.ODOM_FRAME, timeout=self.tf_timeout
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

        if publish_tf:
            static_transform_broadcaster.sendTransform(
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
    executor = MultiThreadedExecutor()

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
