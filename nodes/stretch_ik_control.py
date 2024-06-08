"""
This module contains the StretchIKControl class, which allows users to compute Stretch's FK and IK,
command the robot using an inverse Jacobian velocity controller, and command the arm to a desired pose.

Parts of this approach are inspired from:
https://github.com/RCHI-Lab/HAT2/blob/main/driver_assistance/da_core/scripts/stretch_ik_solver.py
"""
# Standard Imports
import asyncio
import threading
from enum import Enum
from typing import Callable, Dict, Generator, List, Optional, Tuple, Union

# Third-Party Imports
import numpy as np
import numpy.typing as npt
import pinocchio
import rclpy
import tf2_py as tf2
import tf2_ros
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, Twist
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from stretch.motion.pinocchio_ik_solver import PinocchioIKSolver
from stretch_body.robot_params import RobotParams
from tf2_geometry_msgs import PoseStamped
from tf_transformations import (
    euler_from_quaternion,
    quaternion_inverse,
    quaternion_matrix,
    quaternion_multiply,
)
from trajectory_msgs.msg import JointTrajectoryPoint
from urdf import treeFromString

# Local Imports


class ControlMode(Enum):
    """
    The control modes for the Stretch Driver.
    """

    # POSITION = "position"
    NAVIGATION = "navigation"


class SpeedProfile(Enum):
    """
    The speed profile to use to get max velocities and accelerations.
    """

    SLOW = "slow"
    DEFAULT = "default"
    FAST = "fast"
    MAX = "max"


class TerminationCriteria(Enum):
    """
    The termination criteria for the inverse Jacobian controller.
    """

    # Terminate when the commanded velocity is (near) zero.
    ZERO_VEL = "zero_vel"
    # Terminate when the error between the end-effector pose and the goal pose is (near) zero.
    # NOTE: This will not converge if the goal pose is unreachable.
    ZERO_ERR = "zero_err"
    # Stop commanding each joint once it has overshot its goal position.
    # Terminate once all joints have overshot their goal positions.
    OVERSHOOT = "overshoot"


class ActionRetval(Enum):
    """
    The return values for the motion actions.
    """

    CONTINUE = 0
    SUCCESS = 1
    FAILURE = 2


class StretchIKControl:
    """
    The StretchIKControl class allows users to compute Stretch's FK and IK, command the robot using an inverse
    Jacobian velocity controller, and command the arm to a desired pose.
    """

    # Frames of reference
    FRAME_BASE_LINK = "base_link"
    FRAME_END_EFFECTOR_LINK = "link_grasp_center"
    FRAME_ODOM = "odom"

    # Joint names
    JOINT_BASE_ROTATION = "joint_mobile_base_rotation"
    JOINT_ARM_LIFT = "joint_lift"
    JOINTS_ARM = [
        "joint_arm_l3",  # closest from base
        "joint_arm_l2",
        "joint_arm_l1",
        "joint_arm_l0",  # furthest from base
    ]
    JOINT_COMBINED_ARM = "joint_arm"
    JOINT_WRIST_YAW = "joint_wrist_yaw"
    JOINT_WRIST_PITCH = "joint_wrist_pitch"
    JOINT_WRIST_ROLL = "joint_wrist_roll"
    JOINT_GRIPPER = "joint_gripper_finger_left"

    def __init__(
        self,
        node: Node,
        tf_buffer: tf2_ros.Buffer,
        urdf_path: str,
        speed_profile: SpeedProfile = SpeedProfile.FAST,  # TODO: consider allowing users to set this in the message!
    ):
        """
        Create the StretchIKControl object.

        Parameters:
        ----------
        node: The ROS2 node that will run the controller.
        tf_buffer: The tf2 buffer to use to transform poses.
        urdf_path: The path to the URDF file for the robot.
        speed_profile: The speed profile to use to get max velocities and accelerations.
        """
        # Store the parameters
        self.node = node
        self.tf_buffer = tf_buffer
        self.urdf_path = urdf_path
        self.speed_profile = speed_profile
        self.initialized = False

    def initialize(self):
        """
        Initialize the StretchIKControl object.
        """
        # Load the IK solver.
        # We restrict IK to the base rotation, arm lift, and arm joints,
        # to avoid non-ideal IK solutions that either take advantage of
        # joint redundancies (e.g., base rotation and wrist yaw) or get
        # stuck in the gimbal lock that occurs when the wrist pitch is
        # -pi/2. Note that this also means we can only control the below
        # joints with inverse jacobian control.
        self.controllable_joints = [
            self.JOINT_BASE_ROTATION,
            self.JOINT_ARM_LIFT,
            self.JOINTS_ARM[
                -1
            ],  # The URDF uses the most distal of the telescoping joints
        ]
        self.ik_solver = PinocchioIKSolver(
            self.urdf_path, self.FRAME_END_EFFECTOR_LINK, self.controllable_joints
        )
        self.all_joints = [
            self.ik_solver.model.names[i + 1] for i in range(self.ik_solver.model.nq)
        ]
        # Store the control gains
        # NOTE: The CMU implementation had controller gains of 0.2 on the
        # telescoping arm joints and 1.0 otherwise, but for this implementation
        # 1.0 everywhere seems to work fine.
        self.K = np.eye(len(self.all_joints), dtype=np.float64)
        base_rotation_i = self.all_joints.index(self.JOINT_BASE_ROTATION)
        self.K[base_rotation_i, base_rotation_i] = 3.0

        # Subscribe to the robot's joint limits and state
        self.latest_joint_limits_lock = threading.Lock()
        self.latest_joint_limits = None
        self.joint_limits_sub = self.node.create_subscription(
            JointState,
            "/joint_limits",
            self.__joint_limits_cb,
            qos_profile=1,
            # callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.latest_joint_state_lock = threading.Lock()
        self.latest_joint_state = None
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self.__joint_state_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Get the joint limits
        ok = self.__load_joint_limits(speed_profile=self.speed_profile)
        if not ok:
            return False

        # Create service clients to switch control modes
        self.switch_to_position_client = self.node.create_client(
            Trigger,
            "/switch_to_position_mode",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.switch_to_navigation_client = self.node.create_client(
            Trigger,
            "/switch_to_navigation_mode",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Create a publisher and action client to command robot motion
        self.base_vel_pub = self.node.create_publisher(
            Twist,
            "/stretch/cmd_vel",
            qos_profile=1,
        )
        self.arm_client_future = None
        self.arm_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.initialized = True
        return True

    def __load_joint_limits(self, speed_profile: SpeedProfile) -> bool:
        """
        Loads the position and velocity limits for the robot's joints.

        Note that we load a single limit for the arm, as opposed to separate limits
        per telescoping joint. Also note that this function blocks until the service
        to get joint limits is available.

        Parameters
        ----------
        speed_profile: The speed profile to use to get max velocities and accelerations.

        Returns
        -------
        bool: True if the joint limits were successfully loaded.
        """
        # Invoke the service to get joint limits. Note that this service
        # does not actually return the joint limits. Rather, it publishes the joint
        # limits to the /joint_limits topic.
        get_joint_limits_client = self.node.create_client(
            Trigger,
            "get_joint_states",
            # callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.node.get_logger().info("Waiting for the get joint states service...")
        get_joint_limits_client.wait_for_service()  # TODO: This can block!
        response = get_joint_limits_client.call(Trigger.Request())
        if not response.success:
            self.node.get_logger().error("Failed to get joint limits.")
            return False
        self.node.get_logger().info("Received the joint limits.")

        # Get the robot parameters
        robot_params = RobotParams().get_params()[1]
        speed_profile_str = speed_profile.value
        speed_profile_slow_str = SpeedProfile.SLOW.value
        self.joint_pos_lim: Dict[str, Tuple[float, float]] = {}
        self.joint_vel_abs_lim: Dict[str, Tuple[float, float]] = {}

        # Get the velocity limits for the controllable joints
        for i, joint_name in enumerate(self.controllable_joints):
            # Get the max velocity
            if joint_name == self.JOINT_BASE_ROTATION:
                # From https://github.com/hello-robot/stretch_visual_servoing/blob/f99342/normalized_velocity_control.py#L21
                max_abs_vel = 0.5  # rad/s
                min_abs_vel = 0.05  # rad/s
            elif joint_name == self.JOINT_ARM_LIFT:
                max_abs_vel = robot_params["lift"]["motion"][speed_profile_str]["vel_m"]
                min_abs_vel = robot_params["lift"]["motion"][speed_profile_slow_str][
                    "vel_m"
                ]
            elif joint_name == self.JOINTS_ARM[-1]:
                max_abs_vel = robot_params["arm"]["motion"][speed_profile_str]["vel_m"]
                min_abs_vel = robot_params["arm"]["motion"][speed_profile_slow_str][
                    "vel_m"
                ]
            elif joint_name == self.JOINT_WRIST_YAW:
                max_abs_vel = robot_params["wrist_yaw"]["motion"][speed_profile_str][
                    "vel"
                ]
                min_abs_vel = robot_params["wrist_yaw"]["motion"][
                    speed_profile_slow_str
                ]["vel"]
            elif joint_name == self.JOINT_WRIST_PITCH:
                max_abs_vel = robot_params["wrist_pitch"]["motion"][speed_profile_str][
                    "vel"
                ]
                min_abs_vel = robot_params["wrist_pitch"]["motion"][
                    speed_profile_slow_str
                ]["vel"]
            elif joint_name == self.JOINT_WRIST_ROLL:
                max_abs_vel = robot_params["wrist_roll"]["motion"][speed_profile_str][
                    "vel"
                ]
                min_abs_vel = robot_params["wrist_roll"]["motion"][
                    speed_profile_slow_str
                ]["vel"]
            else:
                self.node.get_logger().warn(f"Unknown joint name: {joint_name}")
                max_abs_vel = 0.0
                min_abs_vel = 0.0
            self.joint_vel_abs_lim[joint_name] = (min_abs_vel, max_abs_vel)

        # Get the position limits for all joints.
        with self.latest_joint_limits_lock:
            joint_pos_lim = self.latest_joint_limits
        if joint_pos_lim is None:
            self.node.get_logger().error("Failed to get joint limits.")
            return False

        for joint_name, (min_pos, max_pos) in joint_pos_lim.items():
            if joint_name == self.JOINT_COMBINED_ARM:
                joint_name = self.JOINTS_ARM[-1]
            self.joint_pos_lim[joint_name] = (min_pos, max_pos)

        return True

    def __joint_state_cb(self, msg: JointState) -> None:
        """
        Callback for the joint state subscriber. Save the latest joint state message.

        Note that we set any joint not in the message, e.g., "joint_base_translation" and
        "joint_base_revolution", to 0.0. The reason this works is that we represent the
        end effector goal in the odom frame. As the base moves, the base link moves relative
        to the odom frame, which means the end effector goal gets closer to the base link.
        Hence, it is unnecessary to set non-zero joint values for the base translation and
        revolution joints, as doing so would double-count base motion.

        Parameters
        ----------
        msg: The joint state message.
        """
        with self.latest_joint_state_lock:
            self.latest_joint_state = {
                msg.name[i]: msg.position[i] for i in range(len(msg.name))
            }

    def __joint_limits_cb(self, msg: JointState) -> None:
        """
        Callback for the joint limits subscriber. Save the latest joint limits message.

        Note that the Stretch driver overloads the JointState message by using
        position to indicate the lower limit and velocity to indicate the upper limit.

        Parameters
        ----------
        msg: The joint limits message.
        """
        with self.latest_joint_limits_lock:
            self.latest_joint_limits = {
                msg.name[i]: (msg.position[i], msg.velocity[i])
                for i, joint_name in enumerate(msg.name)
            }

    def move_to_joint_positions(
        self,
        joint_positions: Dict[str, float],
        rate_hz: float = 15.0,
        timeout_secs: float = 10.0,
        check_cancel: Callable[[], bool] = lambda: False,
        threshold_factor: float = 20.0,
    ) -> Generator[ActionRetval, None, None]:
        """
        Move the arm joints to a specific position. This function is closed-loop, and
        terminates once the joint posiiton has been reached.

        Parameters
        ----------
        joint_positions: The target joint positions.
        rate_hz: The rate in Hz at which to control the robot.
        timeout_secs: The timeout in seconds.
        check_cancel: A function that returns True if the action should be cancelled.
        threshold_factor: The factor by which to divide the joint range to get the tolerance.
            A larger number results in a smaller tolerance and more precise motion. But
            too large may result in the robot never reaching the target position.

        Returns
        -------
        ActionRetval: The return value for the action.
        """
        # Check if the controller is initialized
        if not self.initialized:
            self.node.get_logger().error("Controller is not initialized.")
            yield ActionRetval.FAILURE
            return

        # Start the timer
        start_time = self.node.get_clock().now()
        timeout = Duration(seconds=timeout_secs)

        # # Convert to the appropriate control mode
        # # TODO: Store the last mode to avoid re-doing this!
        # future = self.__set_control_mode(
        #     ControlMode.NAVIGATION,
        #     timeout=remaining_time(self.node.get_clock().now(), start_time, timeout),
        # )
        # if future is None:
        #     yield ActionRetval.FAILURE
        #     return
        # while not check_cancel():
        #     if future.done():
        #         result = future.result()
        #         if not result.success:
        #             self.node.get_logger().error(
        #                 f"Failed to switch to {control_mode.value} mode."
        #             )
        #             yield ActionRetval.FAILURE
        #             return
        #         break
        #     yield ActionRetval.CONTINUE

        self.node.get_logger().info(f"Joint Positions: {joint_positions}")

        # The duration of each trajectory command
        duration = 1.0  # seconds

        # Command motion until the termination criteria is reached
        rate = self.node.create_rate(rate_hz)
        future = None
        while not check_cancel():
            self.node.get_logger().info("Loop Started...")
            # Leave early if ROS shuts down or timeout is reached
            if not rclpy.ok() or remaining_time(
                self.node.get_clock().now(), start_time, timeout
            ) <= Duration(seconds=0.0):
                yield ActionRetval.FAILURE
                return

            # Command the robot to move to the joint positions
            if future is None:
                # Get the current joint state
                latest_joint_state_combined_arm = self.get_current_joints(
                    combine_arm=True
                )

                # Remove joints that have reached their set point
                joint_positions_cmd = {}
                for joint_name, joint_position in joint_positions.items():
                    joint_err = (
                        joint_position - latest_joint_state_combined_arm[joint_name]
                    )
                    tolerance = 0.05
                    if joint_name in self.joint_pos_lim:
                        min_pos, max_pos = self.joint_pos_lim[joint_name]
                        tolerance = (max_pos - min_pos) / threshold_factor
                    self.node.get_logger().info(
                        f" Joint {joint_name} error: {joint_err}, tolerance: {tolerance}"
                    )
                    if abs(joint_err) > tolerance:
                        joint_positions_cmd[joint_name] = joint_position

                # If no joints to move, break
                if len(joint_positions_cmd) == 0:
                    yield ActionRetval.SUCCESS
                    return

                # Command the robot to move to the joint positions
                future = self.__command_move_to_joint_position(
                    joint_positions_cmd, duration
                )
            # Wait for the previous command to finish
            elif future.done():
                try:
                    ok = future.result()
                    if not ok:
                        raise Exception("Failed to move to joint positions")
                except Exception as e:
                    self.get_logger().error(f"{e}")
                    yield ActionRetval.FAILURE
                    return
                future = None
            else:
                self.node.get_logger().info("Waiting for previous command to finish...")

            yield ActionRetval.CONTINUE

        yield ActionRetval.FAILURE if check_cancel() else ActionRetval.SUCCESS
        return

    def move_to_ee_pose_inverse_jacobian(
        self,
        goal: PoseStamped,
        articulated_joints: List[str],
        termination: TerminationCriteria,
        joint_position_overrides: Dict[str, float] = {},
        rate_hz: float = 15.0,
        timeout_secs: float = 10.0,
        check_cancel: Callable[[], bool] = lambda: False,
        err_callback: Optional[Callable[[npt.NDArray[np.float64]], None]] = None,
        cartesian_mask: Optional[npt.NDArray[np.bool]] = None,
    ) -> Generator[ActionRetval, None, None]:
        """
        Move the end-effector to the goal pose. This function uses closed-loop inverse
        Jacobian control to determine target velocities for the robot's joints.

        Parameters
        ----------
        goal: The goal pose.
        articulated_joints: The articulated joints. This must be a subset of self.articulated_joints.
        termination: The termination criteria.
        joint_position_overrides: Fixed joint positions to use when computing the jacobian.
        rate_hz: The rate in Hz at which to control the robot.
        timeout_secs: The timeout in seconds.
        check_cancel: A function that returns True if the action should be cancelled.
        err_callback: A callback that is called with the error between the end-effector pose
            and the goal pose.
        cartesian_mask: A mask to apply to the Jacobian matrix. If None, the Jacobian matrix
            is not masked.

        Returns
        -------
        ActionRetval: The return value for the action.
        """
        # Check if the controller is initialized
        if not self.initialized:
            self.node.get_logger().error("Controller is not initialized.")
            yield ActionRetval.FAILURE
            return

        # Start the timer
        start_time = self.node.get_clock().now()
        timeout = Duration(seconds=timeout_secs)

        # Check the articulated joints.
        articulated_joints = [
            joint_name
            for joint_name in articulated_joints
            if joint_name in self.controllable_joints
        ]
        if len(articulated_joints) == 0:
            self.node.get_logger().error("No articulated joints specified.")
            yield ActionRetval.FAILURE
            return
        for joint_name in joint_position_overrides.keys():
            if joint_name in articulated_joints:
                self.node.get_logger().error(
                    f"Cannot pass a fixed position for articulated joint {joint_name}."
                )
                yield ActionRetval.FAILURE
                return
        non_articulated_joints_mask = np.array(
            [joint_name not in articulated_joints for joint_name in self.all_joints]
        )

        # Ensure we are only moving either the base (velocity control) or the arm
        # (position control).
        move_base = False
        move_arm = False
        for joint_name in articulated_joints:
            if joint_name == self.JOINT_BASE_ROTATION:
                move_base = True
            else:
                move_arm = True
        if move_base and move_arm:
            self.node.get_logger().error(
                "Cannot articulate base and arm/wrist joints at the same time."
            )
            yield ActionRetval.FAILURE
            return

        # # Convert to the appropriate control mode
        # # TODO: Store the last mode to avoid re-doing this!
        # future = self.__set_control_mode(
        #     ControlMode.POSITION if move_arm else ControlMode.NAVIGATION,
        #     timeout=remaining_time(self.node.get_clock().now(), start_time, timeout),
        # )
        # if future is None:
        #     yield ActionRetval.FAILURE
        #     return
        # while not check_cancel():
        #     if future.done():
        #         result = future.result()
        #         if not result.success:
        #             self.node.get_logger().error(
        #                 f"Failed to switch to {control_mode.value} mode."
        #             )
        #             yield ActionRetval.FAILURE
        #             return
        #         break
        #     yield ActionRetval.CONTINUE

        # Convert the goal pose to the odom frame, so it stays stationary as the robot moves.
        ok, goal_odom = self.__transform_pose(
            goal, self.FRAME_ODOM, start_time, timeout
        )
        if not ok:
            yield ActionRetval.FAILURE
            return

        # Command motion until the termination criteria is reached
        err = None
        vel = None
        while not check_cancel() and not self.__reached_termination(
            termination, err, vel
        ):
            self.node.get_logger().info("Loop Started...")
            # Leave early if ROS shuts down or timeout is reached
            if not rclpy.ok() or remaining_time(
                self.node.get_clock().now(), start_time, timeout
            ) <= Duration(seconds=0.0):
                yield ActionRetval.FAILURE
                return

            # Get the current joint state
            q = self.__get_q(joint_position_overrides, all_joints=True)
            self.node.get_logger().debug(
                f"Joint Positions: {list(zip(self.all_joints, q))}"
            )

            # Get the error between the end-effector pose and the goal pose
            ok, err = self.get_err(
                goal_odom,
                remaining_time(self.node.get_clock().now(), start_time, timeout),
                q_all=q,
            )
            if not ok:
                yield ActionRetval.FAILURE
                return
            if err_callback is not None:
                err_callback(err)
            self.node.get_logger().info(f"Error: {err}")

            # Get the Jacobian matrix for the chain
            J = pinocchio.computeFrameJacobian(
                self.ik_solver.model,
                self.ik_solver.data,
                q,
                self.ik_solver.ee_frame_idx,
                pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            self.node.get_logger().info(f"Jacobian: {J}")

            # Mask the Jacobian matrix to only include the articulated joints
            J[:, non_articulated_joints_mask] = 0.0
            if cartesian_mask is not None:
                J[cartesian_mask, :] = 0.0

            # Calculate the pseudo-inverse of the Jacobian
            J_pinv = np.linalg.pinv(J, rcond=1e-6)
            self.node.get_logger().info(f"Jacobian Pseudo-Inverse: {J_pinv}")

            # Re-mask the pseudo-inverse to address any numerical issues
            J_pinv[non_articulated_joints_mask, :] = 0.0
            if cartesian_mask is not None:
                J_pinv[:, cartesian_mask] = 0.0

            # Calculate the joint velocities
            vel = self.K @ J_pinv @ err
            self.node.get_logger().info(
                f"Joint Velocities: {list(zip(self.controllable_joints, vel))}"
            )

            # Execute the velocities
            self.__execute_velocities(vel, move_base, move_arm, rate_hz)

            # Yield control back to the main action loop
            yield ActionRetval.CONTINUE

        yield ActionRetval.FAILURE if check_cancel() else ActionRetval.SUCCESS
        return

    def set_navigation_mode(self, timeout: Duration, rate_hz: float = 10.0) -> bool:
        """
        Set the control mode to navigation.

        Parameters
        ----------
        timeout: The timeout.
        rate_hz: The rate in Hz at which to control the robot.

        Returns
        -------
        bool: True if the control mode was successfully set.
        """
        start_time = self.node.get_clock().now()
        # Invoke the service
        self.node.get_logger().info(f"Switching to navigation mode...")
        ready = self.switch_to_navigation_client.wait_for_service(
            timeout_sec=timeout.nanoseconds / 1.0e9
        )
        if not ready:
            self.node.get_logger().error(
                f"Service {self.switch_to_navigation_client.srv_name} not available."
            )
            return False
        future = self.switch_to_navigation_client.call_async(Trigger.Request())
        rate = self.node.create_rate(rate_hz)
        while rclpy.ok() and not future.done():
            # Check if we've reached timeout
            if (
                remaining_time(
                    self.node.get_clock().now(), start_time, timeout, return_secs=True
                )
                <= 0.0
            ):
                self.node.get_logger().error(f"Failed to switch to navigation mode.")
                return False
            rate.sleep()
        if future.done():
            result = future.result()
            if not result.success:
                self.node.get_logger().error(
                    f"Failed to switch to {control_mode.value} mode."
                )
                return False
        return True

    def __transform_pose(
        self, pose: PoseStamped, target_frame: str, start_time: Time, timeout: Duration
    ) -> Tuple[bool, PoseStamped]:
        """
        Transform a pose stamped to the target frame.

        Note that this function may destructively modify the input pose.

        Parameters
        ----------
        pose: The pose to transform.
        target_frame: The target frame.
        start_time: The start time.
        timeout: The timeout.

        Returns
        -------
        PoseStamped: The transformed pose.
        """
        try:
            pose_transformed = self.tf_buffer.transform(
                pose,
                target_frame,
                remaining_time(self.node.get_clock().now(), start_time, timeout),
            )
        except (
            tf2.ConnectivityException,
            tf2.ExtrapolationException,
            tf2.InvalidArgumentException,
            tf2.LookupException,
            tf2.TimeoutException,
            tf2.TransformException,
        ) as e:
            self.node.get_logger().error(
                f"Failed to transform pose {pose} to frame {target_frame}: {e}"
            )
            return False, PoseStamped()

        return True, pose_transformed

    def __reached_termination(
        self,
        termination: TerminationCriteria,
        err: Optional[npt.NDArray[np.float64]],
        vel: Optional[npt.NDArray[np.float64]],
    ) -> bool:
        """
        Check if the termination criteria has been reached.

        Parameters
        ----------
        termination: The termination criteria.
        err: The error between the end-effector pose and the goal pose.
        vel: The commanded velocity.

        Returns
        -------
        bool: True if the termination criteria has been reached.
        """

        if err is None or vel is None:
            return False  # Always allow the first iteration
        if termination == TerminationCriteria.ZERO_VEL:
            # We have reached termination when every joint's commanded
            # velocity is lower than their slowest speed (because after
            # that point, they will no longer move).
            retval = True
            for i, joint_name in enumerate(self.controllable_joints):
                abs_vel = np.abs(vel[i])
                if abs_vel > self.joint_vel_abs_lim[joint_name][0]:
                    retval = False
                    break
            self.node.get_logger().debug(f"Reached Termination {retval}")
            return retval
        elif termination == TerminationCriteria.ZERO_ERR:
            return np.allclose(err, 0.0, atol=1.0e-2)
        # TODO: Implement overshoot-based termination.
        else:
            self.node.get_logger().error(f"Unknown termination criteria: {termination}")
            return True  # auto-terminate

    def get_err(
        self,
        goal: PoseStamped,
        timeout: Duration,
        q_all: Optional[npt.NDArray[np.float64]] = None,
    ) -> Tuple[bool, npt.NDArray[np.float64]]:
        """
        Get the error between the goal pose and the current end effector pose.
        Returns the error in base link frame.

        Parameters
        ----------
        goal: The goal end effector pose.
        timeout: The timeout.
        q_all: The joint positions to use to compute the end effector pose.
            If None, use the current joint positions.

        Returns
        -------
        bool: Whether the error was successfully calculated.
        npt.NDArray[np.float64]: The error in base link frame. The error is a 6D vector
            consisting of the translation and rotation errors.
        """
        # Start the timer
        start_time = self.node.get_clock().now()

        # Get the current end effector pose in base frame
        if q_all is None:
            q_all = self.__get_q(all_joints=True)
        self.ik_solver.q_neutral = q_all
        ee_pos, ee_quat = self.ik_solver.compute_fk(config=q_all)

        # Get the goal end effector pose in base frame
        goal.header.stamp = Time()  # Get the most recent transform
        ok, goal_base = self.__transform_pose(
            goal, self.FRAME_BASE_LINK, start_time, timeout
        )
        if not ok:
            return False, np.zeros(6)

        # Get the error in base frame
        err = np.zeros(6, dtype=np.float64)
        err[:3] = np.array(
            [
                goal_base.pose.position.x - ee_pos[0],
                goal_base.pose.position.y - ee_pos[1],
                goal_base.pose.position.z - ee_pos[2],
            ]
        )
        goal_quaternion = [
            goal_base.pose.orientation.x,
            goal_base.pose.orientation.y,
            goal_base.pose.orientation.z,
            goal_base.pose.orientation.w,
        ]
        err_quaternion = quaternion_multiply(
            goal_quaternion, quaternion_inverse(ee_quat)
        )
        yaw, pitch, roll = euler_from_quaternion(
            err_quaternion,
            axes="rzyx",  # https://wiki.ros.org/geometry2/RotationMethods#Fixed_Axis_vs_Euler_Angles
        )
        err[3:] = np.array([roll, pitch, yaw])

        return True, err

    def __execute_velocities(
        self,
        velocities: npt.NDArray[np.float64],
        move_base: bool,
        move_arm: bool,
        rate_hz: float = 10.0,
        forward_project_velocities_secs: float = 0.5,
    ) -> bool:
        """
        Send the velocities to the ROS2 controllers.

        Parameters
        ----------
        joint_velocities: The joint velocities.
        move_base: Whether to move the base.
        move_arm: Whether to move the arm.
        rate_hz: The rate in Hz at which to control the robot.
        forward_project_velocities_secs: How many seconds to forward-project velocities for
            when commanding the arm in position mode.

        Returns
        -------
        bool: True if it successfully sent the velocities to be executed.
        """
        # Check the parameters
        if move_base and move_arm or (not move_base and not move_arm):
            self.node.get_logger().error(
                "Exactly one of the base or the arm must be articulated."
            )
            return False

        # If moving the arm, wait until the previous arm command is done
        if (
            move_arm
            and self.arm_client_future is not None
            and not self.arm_client_future.done()
        ):
            return True

        # Clip the velocities
        joint_velocities = dict(zip(self.controllable_joints, velocities))
        _, clipped_velocities = self.check_velocity_limits(joint_velocities)
        self.node.get_logger().info(f"Clipped Velocities: {clipped_velocities}")

        # Send base commands
        if move_base:
            base_vel = Twist()
            base_vel.angular.z = clipped_velocities[self.JOINT_BASE_ROTATION]
            self.base_vel_pub.publish(base_vel)
            return True

        # Send arm commands
        q = self.__get_q(all_joints=True)
        joint_positions = {}
        for joint_name, vel in clipped_velocities.items():
            if (
                joint_name != self.JOINT_BASE_ROTATION
                and joint_name in self.all_joints
                and not np.isclose(clipped_velocities[joint_name], 0.0)
            ):
                pos = q[self.all_joints.index(joint_name)]
                joint_positions[joint_name] = (
                    pos + vel * forward_project_velocities_secs
                )
        self.arm_client_future = self.__command_move_to_joint_position(
            joint_positions, forward_project_velocities_secs
        )

        return True

    def __command_move_to_joint_position(
        self, joint_positions: Dict[str, float], duration_sec: float
    ) -> Future:
        """
        Commands the robot arm to move to the specified joint positions.

        Parameters
        ----------
        joint_positions: The joint positions.
        duration_sec: The duration in seconds to reach the joint positions.

        Returns
        -------
        Future: The future object.
        """
        # Limit the joint positions
        _, joint_positions = self.check_joint_limits(joint_positions)
        self.node.get_logger().info(f" Commanding arm to {joint_positions}")

        # Replace the distal arm joint with the combined joint
        if self.JOINTS_ARM[-1] in joint_positions:
            joint_positions[self.JOINT_COMBINED_ARM] = joint_positions.pop(
                self.JOINTS_ARM[-1]
            )

        # Create the goal
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.joint_names = list(joint_positions.keys())
        arm_goal.trajectory.points = [JointTrajectoryPoint()]
        arm_goal.trajectory.points[0].positions = list(joint_positions.values())
        arm_goal.trajectory.points[0].time_from_start = Duration(
            seconds=duration_sec,
        ).to_msg()
        self.node.get_logger().info(f" Commanding arm to {arm_goal}")
        return self.arm_client.send_goal_async(arm_goal)

    def check_joint_limits(
        self,
        joint_positions: Dict[str, float],
        clip: bool = True,
    ) -> Tuple[bool, Dict[str, float]]:
        """
        Check if the joint positions are within the limits. Optionally clip the positions.

        Parameters
        ----------
        joint_positions: The joint positions.
        clip: Whether to clip the joint positions to the limits. IF false,
            returns the joint positions as is.

        Returns
        -------
        Tuple[bool, Dict[str, float]]: Whether the joint positions are within the limits
            and the joint positions (clipped if requested).
        """
        # Check and clip the joint positions
        in_limits = True
        clipped_joint_positions = {}
        for joint_name, pos in joint_positions.items():
            if joint_name in self.joint_pos_lim:
                min_pos, max_pos = self.joint_pos_lim[joint_name]
                if pos < min_pos or pos > max_pos:
                    in_limits = False
                if clip:
                    clipped_joint_positions[joint_name] = np.clip(pos, min_pos, max_pos)
            else:
                if (
                    joint_name != self.JOINT_GRIPPER
                    and joint_name != self.JOINT_BASE_ROTATION
                ):
                    self.node.get_logger().warn(
                        f"Unknown joint name: {joint_name}. Won't clip positions."
                    )
                clipped_joint_positions[joint_name] = pos

        return in_limits, clipped_joint_positions if clip else joint_positions

    def check_velocity_limits(
        self,
        joint_velocities: Dict[str, float],
        clip: bool = True,
        zero_thresh: float = 1.0e-3,
    ) -> Tuple[bool, Dict[str, float]]:
        """
        Check if the joint velocities are within the limits. Optionally clip the velocities.

        Parameters
        ----------
        joint_velocities: The joint velocities.
        clip: Whether to clip the joint velocities to the limits.
        zero_thresh: The threshold below which a velocity is considered zero.

        Returns
        -------
        Tuple[bool, Dict[str, float]]: Whether the joint velocities are within the limits
        """
        # Check and clip the joint velocities
        in_limits = True
        clipped_joint_velocities = {}
        for joint_name, vel in joint_velocities.items():
            if joint_name in self.joint_vel_abs_lim:
                min_abs_vel, max_abs_vel = self.joint_vel_abs_lim[joint_name]
                if vel < min_abs_vel or vel > max_abs_vel:
                    in_limits = False
                if clip:
                    if vel >= zero_thresh:
                        clipped_joint_velocities[joint_name] = np.clip(
                            vel, min_abs_vel, max_abs_vel
                        )
                    elif vel <= -zero_thresh:
                        clipped_joint_velocities[joint_name] = np.clip(
                            vel, -max_abs_vel, -min_abs_vel
                        )
                    else:
                        clipped_joint_velocities[joint_name] = 0.0
            else:
                if joint_name != self.JOINT_GRIPPER:
                    self.node.get_logger().warn(
                        f"Unknown joint name: {joint_name}. Won't clip velocities."
                    )
                clipped_joint_velocities[joint_name] = vel

        return in_limits, clipped_joint_velocities if clip else joint_velocities

    def solve_ik(
        self,
        goal: PoseStamped,
        joint_position_overrides: Dict[str, float] = {},
    ) -> Tuple[bool, Dict[str, float]]:
        """
        Solve the inverse kinematics problem.

        Parameters
        ----------
        goal: The goal pose.
        joint_position_overrides: joint positions to use. Any joint positions
            not in here will use the latest joint state.

        Returns
        -------
        Dict[str, float]: The joint positions.
        """
        # Convert the goal to base link frame
        if goal.header.frame_id != self.FRAME_BASE_LINK:
            ok, goal_base = self.__transform_pose(
                goal,
                self.FRAME_BASE_LINK,
                self.node.get_clock().now(),
                Duration(seconds=0.5),
            )
            if not ok:
                return False, {}
        else:
            goal_base = goal

        # Get the inputs
        pos = np.array(
            [
                goal_base.pose.position.x,
                goal_base.pose.position.y,
                goal_base.pose.position.z,
            ]
        )
        quat = np.array(
            [
                goal_base.pose.orientation.x,
                goal_base.pose.orientation.y,
                goal_base.pose.orientation.z,
                goal_base.pose.orientation.w,
            ]
        )
        q_controllable = self.__get_q(joint_position_overrides)
        q_all = self.__get_q(joint_position_overrides, all_joints=True)

        # Set the positions that are used for non-controllable joints
        self.ik_solver.q_neutral = q_all

        # Solve the IK problem
        q, success, _ = self.ik_solver.compute_ik(pos, quat, q_init=q_controllable)
        if success:
            # Check whether the controllable joints are within their limits
            joint_positions = dict(zip(self.controllable_joints, q))
            within_limits, _ = self.check_joint_limits(joint_positions, clip=False)
            if within_limits:
                return True, joint_positions
            else:
                return False, joint_positions

        return False, {}

    def get_current_joints(self, combine_arm: bool = True) -> Dict[str, float]:
        """
        Get the current states of the controllable joints.

        Parameters
        ----------
        combine_arm: Whether to combine the telescoping joints into a single joint.

        Returns
        -------
        Dict[str, float]: The joint positions
        """
        # Get the latest joints
        with self.latest_joint_state_lock:
            latest_joint_state = self.latest_joint_state

        # Get the positions
        joint_positions = {self.JOINTS_ARM[-1]: 0.0}
        for joint_name in latest_joint_state:
            pos = latest_joint_state[joint_name]
            if combine_arm and joint_name in self.JOINTS_ARM:
                joint_positions[self.JOINTS_ARM[-1]] += pos
            else:
                joint_positions[joint_name] = pos

        return joint_positions

    def __get_q(
        self, joint_position_overrides: Dict[str, float] = {}, all_joints: bool = False
    ) -> npt.NDArray[np.float64]:
        """
        Get the current states of the controllable joints.

        Parameters
        ----------
        joint_position_overrides: override the joint positions with these values.
        all_joints: whether to use all joints or only the controllable ones.

        Returns
        -------
        npt.NDArray: The joint positions
        """
        # Determine which joints we are returning
        if all_joints:
            joints = self.all_joints
        else:
            joints = self.controllable_joints

        # Get the latest joints
        latest_joint_state_combined_arm = self.get_current_joints(combine_arm=True)

        # Get the positions
        joint_positions = []
        for joint_name in joints:
            if joint_name in joint_position_overrides:
                joint_positions.append(joint_position_overrides[joint_name])
            elif joint_name in latest_joint_state_combined_arm:
                joint_positions.append(latest_joint_state_combined_arm[joint_name])
            else:
                # Set dummy joints to 0.0
                joint_positions.append(0.0)
        return np.array(joint_positions, dtype=np.float64)


def remaining_time(
    now: Time, start_time: Time, timeout: Duration, return_secs: bool = False
) -> Duration:
    """
    Get the remaining time until the timeout.

    Parameters
    ----------
    now: The current time.
    start_time: The start time.
    timeout: The timeout.
    return_secs: Whether to return the remaining time in seconds.

    Returns
    -------
    Duration: The remaining time.
    """
    elapsed_time = now - start_time
    diff = Duration(nanoseconds=timeout.nanoseconds - elapsed_time.nanoseconds)
    if return_secs:
        return diff.nanoseconds / 1.0e9
    return diff
