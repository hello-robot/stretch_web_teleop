#!/usr/bin/env python3

# Standard Imports
from __future__ import annotations  # Required for type hinting MoveToPregraspState.next

import threading
from enum import Enum
from typing import Dict, List, Optional, Tuple

# Third-Party Imports
import numpy as np
import numpy.typing as npt
import rclpy
import ros2_numpy
import tf2_py as tf2
import tf2_ros
from configure_video_streams import ConfigureVideoStreams
from geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped, Vector3
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Header
from stretch_controls import StretchControls, TerminationCriteria, remaining_time
from tf2_geometry_msgs import PoseStamped
from tf_transformations import quaternion_about_axis, quaternion_multiply

# Local Imports
from stretch_web_teleop.action import MoveToPregrasp as MoveToPregraspAction


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


class MoveToPregrasp:
    """
    This class exposes an action server that takes in the
    (x, y) pixel coordinates of an operator's click on the Realsense
    camera feed. It then moves the robot so its end-effector is
    aligned with the clicked pixel, making it easy for the user to grasp
    the object the pixel is a part of.
    """

    # TODO: Eventually move this and all frames to some definitions/constants file.
    BASE_LINK = "base_link"
    END_EFFECTOR_LINK = "link_grasp_center"
    ODOM_FRAME = "odom"

    DISTANCE_TO_OBJECT = 0.1  # meters

    STOW_CONFIGURATION_ARM_LENGTH = {
        StretchControls.JOINTS_ARM[-1]: 0.0,
    }
    STOW_CONFIGURATION_ARM_LIFT = {
        StretchControls.JOINT_ARM_LIFT: 0.35,
    }
    STOW_CONFIGURATION_WRIST = {
        StretchControls.JOINT_GRIPPER: 0.0,
        # Wrist rotation should match src/shared/util.tsx
        StretchControls.JOINT_WRIST_ROLL: 0.0,
        StretchControls.JOINT_WRIST_PITCH: -0.497,
        StretchControls.JOINT_WRIST_YAW: 3.19579,
    }

    def __init__(
        self,
        node: Node,
        video_streams: ConfigureVideoStreams,
        stretch_controls: StretchControls,
        tf_buffer: tf2_ros.Buffer,
        tf_timeout_secs: float = 0.5,
        action_timeout_secs: float = 60.0,  # TODO: lower!
    ):
        """
        Initialize the MoveToPregraspNode

        Parameters
        ----------
        node: The ROS node.
        video_streams: The video streams.
        stretch_controls: The Stretch controller.
        tf_buffer: The TF buffer.
        tf_timeout_secs: The number of seconds to cache transforms for.
        action_timeout_secs: The number of seconds before the action times out.
        """
        # Store the parameters
        self.node = node
        self.video_streams = video_streams
        self.stretch_controls = stretch_controls
        self.tf_buffer = tf_buffer
        self.tf_timeout = Duration(seconds=tf_timeout_secs)
        self.action_timeout = Duration(seconds=action_timeout_secs)

        # Create the shared resource to ensure that the action server rejects all
        # new goals while a goal is currently active.
        self.active_goal_request_lock = threading.Lock()
        self.active_goal_request = None

        # Create the action server
        self.action_server = ActionServer(
            self.node,
            MoveToPregraspAction,
            "move_to_pregrasp",
            self.__execute_callback,
            goal_callback=self.__goal_callback,
            cancel_callback=self.__cancel_callback,
            # callback_group=ReentrantCallbackGroup(),
            # lambda goal_handle: self.node.get_logger().info("Executing goal."),
            # goal_callback=lambda goal_handle: self.node.get_logger().info("Received goal."),
            # cancel_callback=lambda goal_handle: self.node.get_logger().info("Cancelled goal."),
        )

    def initialize(self) -> bool:
        """
        Initialize parts of this class that require ROS to be spinning in the background.
        """
        return True

    def __goal_callback(self, goal_request: MoveToPregraspAction.Goal) -> GoalResponse:
        """
        Accept a goal if this action does not already have an active goal, else reject.

        Parameters
        ----------
        goal_request: The goal request message.
        """
        self.node.get_logger().info(f"Received request {goal_request}")

        # Reject if the controls have not yet been initialized
        if not self.stretch_controls.initialized:
            self.node.get_logger().info(
                "Rejecting goal request since the Stretch controls have not been initialized yet"
            )
            return GoalResponse.REJECT

        # Reject the goal if no camera info has been received yet
        P = self.video_streams.get_realsense_projection_matrix()
        if P is None:
            self.node.get_logger().info(
                "Rejecting goal request since no camera info has been received yet"
            )
            return GoalResponse.REJECT

        # Reject the goal if no Realsense messages have been received yet
        latest_realsense_msgs = self.video_streams.get_latest_realsense_msgs()
        if latest_realsense_msgs is None:
            self.node.get_logger().info(
                "Rejecting goal request since no Realsense messages have been received yet"
            )
            return GoalResponse.REJECT

        # Reject the goal is there is already an active goal
        with self.active_goal_request_lock:
            if self.active_goal_request is not None:
                self.node.get_logger().info(
                    "Rejecting goal request since there is already an active one"
                )
                return GoalResponse.REJECT

        # Accept the goal
        self.node.get_logger().info("Accepting goal request")
        self.active_goal_request = goal_request
        return GoalResponse.ACCEPT

    def __cancel_callback(self, _: ServerGoalHandle) -> CancelResponse:
        """
        Always accept client requests to cancel the active goal.

        Parameters
        ----------
        goal_handle: The goal handle.
        """
        self.node.get_logger().info("Received cancel request, accepting")
        return CancelResponse.ACCEPT

    async def __execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> MoveToPregraspAction.Result:
        """
        Execute the goal, by rotating the robot's base and adjusting the arm lift/length
        to align the end-effector with the clicked pixel.

        Parameters
        ----------
        goal_handle: The goal handle.

        Returns
        -------
        MoveToPregraspAction.Result: The result message.
        """
        self.node.get_logger().info(f"Got request {goal_handle.request}")

        # Start the timer
        start_time = self.node.get_clock().now()

        # Initialize the feedback
        feedback = MoveToPregraspAction.Feedback()
        feedback.initial_distance_m = -1.0

        def distance_callback(err: npt.NDArray[np.float32]) -> None:
            self.node.get_logger().info(f" Distance error: {err}")
            distance = np.linalg.norm(err[:3])
            if feedback.initial_distance_m < 0.0:
                feedback.initial_distance_m = distance
            else:
                feedback.remaining_distance_m = distance

        # Functions to cleanup the action
        future: Optional[Future] = None

        def cleanup() -> None:
            self.active_goal_request = None
            if future is not None:
                future.cancel()

        def action_error_callback(
            error_msg: str = "Goal failed",
            status: int = MoveToPregraspAction.Result.STATUS_FAILURE,
        ) -> MoveToPregraspAction.Result:
            self.node.get_logger().error(error_msg)
            goal_handle.abort()
            cleanup()
            return MoveToPregraspAction.Result(status=status)

        def action_success_callback(
            success_msg: str = "Goal succeeded",
        ) -> MoveToPregraspAction.Result:
            self.node.get_logger().info(success_msg)
            goal_handle.succeed()
            cleanup()
            return MoveToPregraspAction.Result(
                status=MoveToPregraspAction.Result.STATUS_SUCCESS
            )

        def action_cancel_callback(
            cancel_msg: str = "Goal canceled",
        ) -> MoveToPregraspAction.Result:
            self.node.get_logger().info(cancel_msg)
            goal_handle.canceled()
            cleanup()
            return MoveToPregraspAction.Result(
                status=MoveToPregraspAction.Result.STATUS_CANCELLED
            )

        # Undo any transformation that were applied to the raw camera image before sending it
        # to the web app
        raw_scaled_u, raw_scaled_v = (
            goal_handle.request.scaled_u,
            goal_handle.request.scaled_v,
        )
        # TODO: The below has an issue!
        ok, u, v = self.video_streams.inverse_transform_pixel(
            raw_scaled_u, raw_scaled_v, ["realsense", "default"], scaled=True
        )
        if not ok:
            return action_error_callback(
                "Failed to inverse transform the clicked pixel"
            )
        self.node.get_logger().info(
            f"Clicked pixel after inverse transform (camera frame): {(u, v)}"
        )

        # Get the latest Realsense messages
        latest_realsense_msgs = self.video_streams.get_latest_realsense_msgs()
        if latest_realsense_msgs is None:
            return action_error_callback("Failed to get latest Realsense messages")
        rgb_msg, pointcloud_msg = latest_realsense_msgs
        pointcloud = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(
            pointcloud_msg
        )  # N x 3 array

        # Deproject the clicked pixel to get the 3D coordinates of the clicked point
        x, y, z = self.__deproject_pixel_to_point(u, v, pointcloud)
        self.node.get_logger().info(
            f"Closest point to clicked pixel (camera frame): {(x, y, z)}"
        )

        # Determine how the robot should orient its gripper to align with the clicked pixel
        horizontal_grasp = self.__get_grasp_orientation(goal_handle.request)

        # Get the goal end effector pose
        ok, goal_pose = self.__get_goal_pose(
            x, y, z, pointcloud_msg.header, horizontal_grasp, publish_tf=True
        )
        if not ok:
            return action_error_callback("Failed to get goal pose")
        self.node.get_logger().info(f"Goal Pose: {goal_pose}")
        return action_success_callback()

        # Verify the goal is reachable
        wrist_rotation = self.__get_wrist_rotation(horizontal_grasp)
        reachable, ik_solution = self.stretch_controls.solve_ik(
            goal_pose, joint_position_overrides=wrist_rotation
        )
        if not reachable:
            return action_error_callback(
                f"Goal pose is not reachable {ik_solution}",
                status=MoveToPregraspAction.Result.STATUS_GOAL_NOT_REACHABLE,
            )

        # Get the current joints
        init_joints = self.stretch_controls.get_current_joints()

        # Get the states.
        # If the robot is in a vertical grasp position and the arm needs to descend,
        # lengthn the arm before deploying the wrist.
        states = MoveToPregraspState.get_state_machine(
            horizontal_grasp=horizontal_grasp,
            init_lift_near_base=init_joints[StretchControls.JOINT_ARM_LIFT]
            < self.STOW_CONFIGURATION_ARM_LIFT[StretchControls.JOINT_ARM_LIFT],
            goal_lift_near_base=ik_solution[StretchControls.JOINT_ARM_LIFT]
            < self.STOW_CONFIGURATION_ARM_LIFT[StretchControls.JOINT_ARM_LIFT],
        )
        self.node.get_logger().info(f" States: {states}")

        state_i = 0
        terminate_future = False
        rate = self.node.create_rate(10.0)
        while rclpy.ok():
            state = states[state_i]
            self.node.get_logger().info(f"State: {state}", throttle_duration_sec=1.0)
            # Check if a cancel has been requested
            if goal_handle.is_cancel_requested:
                return action_cancel_callback()
            # Check if the action has timed out
            if (self.node.get_clock().now() - start_time) > self.action_timeout:
                return action_error_callback(
                    "Goal timed out", status=MoveToPregraspAction.Result.STATUS_TIMEOUT
                )

            # Get the IK solution if necessary
            if state.use_ik():
                reachable, ik_solution = self.stretch_controls.solve_ik(
                    goal_pose,
                    joint_position_overrides=wrist_rotation,
                )
                if not reachable:
                    return action_error_callback(
                        f"Failed to solve IK {ik_solution}",
                        status=MoveToPregraspAction.Result.STATUS_GOAL_NOT_REACHABLE,
                    )

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
                    joints_for_velocity_control += [StretchControls.JOINT_BASE_ROTATION]
                    joint_position_overrides.update(
                        self.__get_wrist_rotation(horizontal_grasp)
                    )
                elif state == MoveToPregraspState.LIFT_ARM:
                    joints_for_position_control[
                        StretchControls.JOINT_ARM_LIFT
                    ] = ik_solution[StretchControls.JOINT_ARM_LIFT]
                elif state == MoveToPregraspState.MOVE_WRIST:
                    joints_for_position_control[StretchControls.JOINT_GRIPPER] = 0.84
                    joints_for_position_control.update(
                        self.__get_wrist_rotation(horizontal_grasp)
                    )
                elif state == MoveToPregraspState.LENGTHEN_ARM:
                    joints_for_position_control[
                        StretchControls.JOINTS_ARM[-1]
                    ] = ik_solution[StretchControls.JOINTS_ARM[-1]]
                else:  # TERMINAL
                    return action_success_callback()
                if len(joints_for_velocity_control) > 0:
                    future = self.node.executor.create_task(
                        self.stretch_controls.move_to_ee_pose_inverse_jacobian(
                            goal=goal_pose,
                            articulated_joints=joints_for_velocity_control,
                            termination=TerminationCriteria.ZERO_VEL,
                            joint_position_overrides=joint_position_overrides,
                            timeout_secs=remaining_time(
                                self.node.get_clock().now(),
                                start_time,
                                self.action_timeout,
                                return_secs=True,
                            ),
                            check_cancel=lambda: terminate_future,
                            err_callback=distance_callback,
                        )
                    )
                else:
                    future = self.node.executor.create_task(
                        self.stretch_controls.move_to_joint_positions(
                            joint_positions=joints_for_position_control,
                            timeout_secs=remaining_time(
                                self.node.get_clock().now(),
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
                    return action_error_callback(f"{e}")
                state_i += 1
                future = None

            # Send feedback. If we are not controlling any joints with the inverse
            # Jacobian, then we need to calculate the error here.
            ok, err = self.stretch_controls.get_err(
                goal_pose,
                timeout=remaining_time(
                    self.node.get_clock().now(),
                    start_time,
                    self.action_timeout,
                ),
            )
            if ok:
                distance_callback(err)
            feedback.elapsed_time = (self.node.get_clock().now() - start_time).to_msg()
            goal_handle.publish_feedback(feedback)

            # Sleep
            rate.sleep()

        # The only way to reach here is if rclpy.ok() returns False before another
        # termination condition
        return action_error_callback()

    def __get_grasp_orientation(self, request: MoveToPregraspAction.Goal) -> bool:
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
            == MoveToPregraspAction.Goal.PREGRASP_DIRECTION_HORIZONTAL
        ):
            return True
        elif (
            request.pregrasp_direction
            == MoveToPregraspAction.Goal.PREGRASP_DIRECTION_VERTICAL
        ):
            return False
        else:  # auto
            self.node.get_logger().warn(
                "Auto grasp not implemented yet. Defaulting to horizontal grasp."
            )
            return True

    def __get_wrist_rotation(self, horizontal_grasp: bool) -> Dict[str, float]:
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
                StretchControls.JOINT_WRIST_YAW: 0.0,
                StretchControls.JOINT_WRIST_PITCH: 0.0,
                StretchControls.JOINT_WRIST_ROLL: 0.0,
            }
        else:
            return {
                StretchControls.JOINT_WRIST_YAW: 0.0,
                StretchControls.JOINT_WRIST_PITCH: -np.pi / 2.0,
                StretchControls.JOINT_WRIST_ROLL: 0.0,
            }

    def __deproject_pixel_to_point(
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
        # We are guaranteed this is not None since the goal would have been rejected otherwise
        P = self.video_streams.get_realsense_projection_matrix()

        # Get the ray from the camera origin to the clicked point
        ray_dir = np.linalg.pinv(P)[:3, :] @ np.array([u, v, 1])
        ray_dir /= np.linalg.norm(ray_dir)

        # Find the point that is closest to the ray
        p, r = pointcloud, ray_dir
        closest_point_idx = np.argmin(
            np.linalg.norm(
                p - np.multiply((p @ r).reshape((-1, 1)), r.reshape((1, 3))), axis=1
            )
        )

        return p[closest_point_idx]

    def __get_goal_pose(
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
            static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)

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
            self.node.get_logger().error(
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
                self.node.get_logger().error(
                    f"Clicked point is too close to the robot: {xy_dist} < {self.DISTANCE_TO_OBJECT}"
                )
                return False, PoseStamped()
            xy = xy / xy_dist * (xy_dist - self.DISTANCE_TO_OBJECT)
            goal_pose_base.pose.position.x, goal_pose_base.pose.position.y = xy
        else:
            goal_pose_base.pose.position.z += self.DISTANCE_TO_OBJECT

        self.node.get_logger().info(f"Goal pose in base link frame: {goal_pose_base}")

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
            self.node.get_logger().error(
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
