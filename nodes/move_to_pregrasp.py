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

    STOW_ARM_LENGTH = 0
    STOW_WRIST = 1
    STOW_ARM_LIFT = 2
    ROTATE_BASE = 3
    LIFT_ARM = 4
    MOVE_WRIST = 5
    LENGTHEN_ARM = 6
    TERMINAL = 7

    @staticmethod
    def get_state_machine(
        horizontal_grasp: bool, init_lift_near_base: bool, goal_lift_near_base: bool
    ) -> List[MoveToPregraspState]:
        """
        Get the default state machine.

        Parameters
        ----------
        horizontal_grasp: Whether the robot will be grasping the object horizontally
            (True) or vertically (False).
        init_lift_near_base: Whether the robot's arm is near the base at the start.
        goal_lift_near_base: Whether the robot's arm should be near the base at the end.

        Returns
        -------
        List[MoveToPregraspState]: The default state machine.
        """
        states = []
        states.append(MoveToPregraspState.STOW_ARM_LENGTH)
        # If the arm lift is currently near the base, raise it up so there is space to stow the wrist.
        if init_lift_near_base:
            states.append(MoveToPregraspState.STOW_ARM_LIFT)
        states.append(MoveToPregraspState.STOW_WRIST)
        states.append(MoveToPregraspState.ROTATE_BASE)
        # If the goal is near the base and we're doing a vertical grasp, lengthen the arm before deploying the wrist.
        if goal_lift_near_base and not horizontal_grasp:
            states.append(MoveToPregraspState.LENGTHEN_ARM)
            states.append(MoveToPregraspState.MOVE_WRIST)
            states.append(MoveToPregraspState.LIFT_ARM)
        else:
            states.append(MoveToPregraspState.LIFT_ARM)
            states.append(MoveToPregraspState.MOVE_WRIST)
            states.append(MoveToPregraspState.LENGTHEN_ARM)
        states.append(MoveToPregraspState.TERMINAL)

        return states

    def use_ik(self) -> bool:
        """
        Whether the state requires us to compute the arm's IK.

        Returns
        -------
        bool: Whether the state requires the inverse jacobian controller.
        """
        return self in [
            MoveToPregraspState.LIFT_ARM,
            MoveToPregraspState.LENGTHEN_ARM,
        ]


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

    STOW_CONFIGURATION_ARM_LENGTH = {
        StretchIKControl.JOINTS_ARM[-1]: 0.0,
    }
    STOW_CONFIGURATION_ARM_LIFT = {
        StretchIKControl.JOINT_ARM_LIFT: 0.35,
    }
    STOW_CONFIGURATION_WRIST = {
        StretchIKControl.JOINT_GRIPPER: 0.0,
        # Wrist rotation should match src/shared/util.tsx
        StretchIKControl.JOINT_WRIST_ROLL: 0.0,
        StretchIKControl.JOINT_WRIST_PITCH: -0.497,
        StretchIKControl.JOINT_WRIST_YAW: 3.19579,
    }

    def __init__(
        self,
        image_params_file: str,
        tf_timeout_secs: float = 0.5,
        action_timeout_secs: float = 60.0,  # TODO: lower!
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
        # TODO: Figure out where the URDF should go!
        urdf_abs_path = "/home/hello-robot/stretchpy/src/stretch/motion/stretch_base_rotation_ik.urdf"
        self.controller = StretchIKControl(
            self, tf_buffer=self.tf_buffer, urdf_path=urdf_abs_path
        )

        # Subscribe to the Realsense's RGB, pointcloud, and camera info feeds
        self.latest_realsense_msgs_lock = threading.Lock()
        self.latest_realsense_msgs: Optional[Tuple[Image, PointCloud2]] = None
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
            self.get_logger().info(f" Distance error: {err}")
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
        horizontal_grasp = self.get_grasp_orientation(goal_handle.request)

        # Get the goal end effector pose
        ok, goal_pose = self.get_goal_pose(
            x, y, z, pointcloud_msg.header, horizontal_grasp, publish_tf=True
        )
        if not ok:
            self.get_logger().error("Failed to get goal pose")
            goal_handle.abort()
            self.active_goal_request = None
            return MoveToPregrasp.Result()
        self.get_logger().info(f"Goal Pose: {goal_pose}")

        # Verify the goal is reachable
        wrist_rotation = self.get_wrist_rotation(horizontal_grasp)
        reachable, ik_solution = self.controller.solve_ik(
            goal_pose, joint_position_overrides=wrist_rotation
        )
        if not reachable:
            self.get_logger().error(f"Goal pose is not reachable {ik_solution}")
            goal_handle.abort()
            self.active_goal_request = None
            return MoveToPregrasp.Result(
                status=MoveToPregrasp.Result.STATUS_GOAL_NOT_REACHABLE
            )

        # Get the current joints
        init_joints = self.controller.get_current_joints()

        # Get the states.
        # If the robot is in a vertical grasp position and the arm needs to descend,
        # lengthn the arm before deploying the wrist.
        states = MoveToPregraspState.get_state_machine(
            horizontal_grasp=horizontal_grasp,
            init_lift_near_base=init_joints[StretchIKControl.JOINT_ARM_LIFT]
            < self.STOW_CONFIGURATION_ARM_LIFT[StretchIKControl.JOINT_ARM_LIFT],
            goal_lift_near_base=ik_solution[StretchIKControl.JOINT_ARM_LIFT]
            < self.STOW_CONFIGURATION_ARM_LIFT[StretchIKControl.JOINT_ARM_LIFT],
        )
        self.get_logger().info(f" States: {states}")

        state_i = 0
        future = None
        terminate_future = False
        result = MoveToPregrasp.Result()
        rate = self.create_rate(10.0)
        while rclpy.ok():
            state = states[state_i]
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

            # Get the IK solution if necessary
            if state.use_ik():
                reachable, ik_solution = self.controller.solve_ik(
                    goal_pose,
                    joint_position_overrides=wrist_rotation,
                )
                if not reachable:
                    self.get_logger().error(f"Failed to solve IK {ik_solution}")
                    result.status = MoveToPregrasp.Result.STATUS_GOAL_NOT_REACHABLE
                    goal_handle.abort()
                    break

            # Move the robot
            if future is None:
                joints_for_velocity_control = []
                joint_position_overrides = {}
                joints_for_position_control = {}
                if state == MoveToPregraspState.STOW_ARM_LENGTH:
                    joints_for_position_control.update(
                        self.STOW_CONFIGURATION_ARM_LENGTH
                    )
                elif state == MoveToPregraspState.STOW_ARM_LIFT:
                    joints_for_position_control.update(self.STOW_CONFIGURATION_ARM_LIFT)
                elif state == MoveToPregraspState.STOW_WRIST:
                    joints_for_position_control.update(self.STOW_CONFIGURATION_WRIST)
                elif state == MoveToPregraspState.ROTATE_BASE:
                    joints_for_velocity_control += [
                        StretchIKControl.JOINT_BASE_ROTATION
                    ]
                    joint_position_overrides.update(
                        self.get_wrist_rotation(horizontal_grasp)
                    )
                elif state == MoveToPregraspState.LIFT_ARM:
                    joints_for_position_control[
                        StretchIKControl.JOINT_ARM_LIFT
                    ] = ik_solution[StretchIKControl.JOINT_ARM_LIFT]
                elif state == MoveToPregraspState.MOVE_WRIST:
                    joints_for_position_control[StretchIKControl.JOINT_GRIPPER] = 0.84
                    joints_for_position_control.update(
                        self.get_wrist_rotation(horizontal_grasp)
                    )
                elif state == MoveToPregraspState.LENGTHEN_ARM:
                    joints_for_position_control[
                        StretchIKControl.JOINTS_ARM[-1]
                    ] = ik_solution[StretchIKControl.JOINTS_ARM[-1]]
                else:  # TERMINAL
                    self.get_logger().info("Goal succeeded")
                    result.status = MoveToPregrasp.Result.STATUS_SUCCESS
                    goal_handle.succeed()
                    break
                if len(joints_for_velocity_control) > 0:
                    future = self.executor.create_task(
                        self.controller.move_to_ee_pose_inverse_jacobian(
                            goal=goal_pose,
                            articulated_joints=joints_for_velocity_control,
                            termination=TerminationCriteria.ZERO_VEL,
                            joint_position_overrides=joint_position_overrides,
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
                            joint_positions=joints_for_position_control,
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
                state_i += 1
                future = None

            # Send feedback. If we are not controlling any joints with the inverse
            # Jacobian, then we need to calculate the error here.
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

    def get_grasp_orientation(self, request: MoveToPregrasp.Goal) -> bool:
        """
        Get the grasp orientation.

        Parameters
        ----------
        goal_handle: The goal handle.

        Returns
        -------
        bool: Whether the grasp orientation is horizontal.
        """
        if (
            request.pregrasp_direction
            == MoveToPregrasp.Goal.PREGRASP_DIRECTION_HORIZONTAL
        ):
            return True
        elif (
            request.pregrasp_direction
            == MoveToPregrasp.Goal.PREGRASP_DIRECTION_VERTICAL
        ):
            return False
        else:  # auto
            self.get_logger().warn(
                "Auto grasp not implemented yet. Defaulting to horizontal grasp."
            )
            return True

    def get_wrist_rotation(self, horizontal_grasp: bool) -> Dict[str, float]:
        """
        Get the wrist rotation for the pregrasp position.

        Parameters
        ----------
        horizontal_grasp: Whether the pregrasp position is horizontal.

        Returns
        -------
        Dict[str, float]: The wrist rotation.
        """
        if horizontal_grasp:
            return {
                StretchIKControl.JOINT_WRIST_YAW: 0.0,
                StretchIKControl.JOINT_WRIST_PITCH: 0.0,
                StretchIKControl.JOINT_WRIST_ROLL: 0.0,
            }
        else:
            return {
                StretchIKControl.JOINT_WRIST_YAW: 0.0,
                StretchIKControl.JOINT_WRIST_PITCH: -np.pi / 2.0,
                StretchIKControl.JOINT_WRIST_ROLL: 0.0,
            }

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

        self.get_logger().info(f"Goal pose in base link frame: {goal_pose_base}")

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
