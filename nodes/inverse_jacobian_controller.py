"""
This module contains an implementation of inverse Jacobian control
for the Stretch robot. In order to include base rotation, this
controller adds a fake revolute joint between the the base and mast.
This approach is inspired from:
https://github.com/RCHI-Lab/HAT2/blob/main/driver_assistance/da_core/scripts/stretch_ik_solver.py

Note that because the ROS2 interface currently only provides velocity control
for the base and position control for the arm, this module:
  (a) Moves the base separately from the arm, as the drivers do not allow the
      position and velocity hardware interfaces to be claimed simultaneously.
  (b) Forward-projects velocity commands to the arm into position commands.
"""
# Standard Imports
import asyncio
import threading
from enum import Enum
from typing import Dict, List, Optional, Tuple

# Third-Party Imports
import numpy as np
import numpy.typing as npt
import PyKDL as kdl
import rclpy
import tf2_py as tf2
import tf2_ros
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from stretch_body.robot_params import RobotParams
from tf2_geometry_msgs import PoseStamped
from tf_transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
from urdf import treeFromString

# Local Imports


class ControlMode(Enum):
    """
    The control modes for the inverse Jacobian controller.
    """

    POSITION = "position"
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
    # TODO: Consider adding another termination criteria for if the joints don't move for n seconds.
    # This becomes slightly challenging with the base though, since that relies on the odom -> base_link
    # transform.


class InverseJacobianController:
    """
    This class implements an inverse Jacobian controller for the Stretch robot.
    """

    # Frames of reference
    FRAME_BASE_LINK = "base_link"
    FRAME_END_EFFECTOR_LINK = "link_grasp_center"
    FRAME_ODOM = "odom"

    # Joint names
    JOINT_BASE_TRANSLATION = "joint_base_translation"
    JOINT_BASE_REVOLUTION = "joint_base_revolution"
    JOINTS_BASE = [
        JOINT_BASE_TRANSLATION,
        JOINT_BASE_REVOLUTION,
    ]
    JOINT_ARM_LIFT = "joint_lift"
    JOINTS_ARM = [
        "joint_arm_l3",
        "joint_arm_l2",
        "joint_arm_l1",
        "joint_arm_l0",
    ]
    JOINT_COMBINED_ARM = "joint_arm"
    JOINT_WRIST_YAW = "joint_wrist_yaw"
    JOINT_WRIST_PITCH = "joint_wrist_pitch"
    JOINT_WRIST_ROLL = "joint_wrist_roll"

    def __init__(
        self,
        node: Node,
        tf_buffer: tf2_ros.Buffer,
        speed_profile: SpeedProfile = SpeedProfile.FAST,
    ):
        """
        Create the inverse Jacobian controller.

        Parameters
        ----------
        node: The ROS2 node that will run the controller.
        tf_buffer: The tf2 buffer to use to transform poses.
        speed_profile: The speed profile to use to get max velocities and accelerations.
        """
        # Store the parameters
        self.node = node
        self.tf_buffer = tf_buffer
        self.speed_profile = speed_profile
        self.initialized = False

    def initialize(self) -> bool:
        """
        Initialize the inverse Jacobian controller.
        """
        # Load the Jacobian solver
        self.jacobian_joint_order = []
        ok = self.__load_jacobian_solver()
        if not ok:
            return False

        # Subscribe to the robot's joint limits and state
        self.latest_joint_limits_lock = threading.Lock()
        self.latest_joint_limits = None
        self.joint_limits_sub = self.node.create_subscription(
            JointState,
            "/joint_limits",
            self.__joint_limits_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
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

        # Initialize the control mode
        self.control_mode = None

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

    def __load_jacobian_solver(self) -> bool:
        """
        Load the Jacobian solver for the robot.

        Note that this function blocks until the URDF is received.
        """
        # Get the robot URDF.
        robot_state_param_client = self.node.create_client(
            GetParameters,
            "robot_state_publisher/get_parameters",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        # TODO: Sometimes the below wait_for_service blocks indefinetely.
        self.node.get_logger().info("Waiting for the robot state publisher...")
        robot_state_param_client.wait_for_service()
        response = robot_state_param_client.call(
            GetParameters.Request(
                names=["robot_description"],
            )
        )
        self.node.get_logger().info("Received the robot URDF string.")
        urdf_string = response.values[0].string_value
        if len(urdf_string) == 0:
            self.node.get_logger().error("Failed to get robot URDF.")
            return False

        # Load the URDF
        ok, robot = treeFromString(urdf_string)
        if not ok:
            self.node.get_logger().error("Failed to load the URDF")
            return False

        # Get the original chain as specified in the URDF
        orig_chain = robot.getChain(self.FRAME_BASE_LINK, self.FRAME_END_EFFECTOR_LINK)

        # Prepend two "dummy links" to the chain: for base translation and revolution
        chain = kdl.Chain()
        chain.addSegment(
            kdl.Segment(
                name="link_base_translation",
                joint=kdl.Joint(
                    name=self.JOINT_BASE_TRANSLATION,
                    origin=kdl.Vector(0, 0, 0),
                    axis=kdl.Vector(1, 0, 0),
                    type=kdl.Joint.TransAxis,
                ),
                f_tip=kdl.Frame(),
                I=kdl.RigidBodyInertia(
                    m=1.0,
                    oc=kdl.Vector(0, 0, 0),
                    Ic=kdl.RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0),
                ),
            )
        )
        chain.addSegment(
            kdl.Segment(
                name="link_base_revolution",
                joint=kdl.Joint(
                    name=self.JOINT_BASE_REVOLUTION,
                    origin=kdl.Vector(0, 0, 0),
                    axis=kdl.Vector(0, 0, 1),
                    type=kdl.Joint.RotAxis,
                ),
                f_tip=kdl.Frame(),
                I=kdl.RigidBodyInertia(
                    m=1.0,
                    oc=kdl.Vector(0, 0, 0),
                    Ic=kdl.RotationalInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0),
                ),
            )
        )
        chain.addChain(orig_chain)
        self.chain = chain

        # # TODO: Remove!
        # old_jacobian_joint_order = []
        # for i in range(self.chain.getNrOfSegments()):
        #     joint = self.chain.getSegment(i).getJoint()
        #     if joint.getType() != kdl.Joint.Fixed:
        #         old_jacobian_joint_order.append(joint.getName())
        # self.chain = kdl_chain_fix_joints(
        #     self.chain,
        #     [
        #         joint_name
        #         for joint_name in old_jacobian_joint_order
        #         if joint_name not in self.articulated_joints
        #     ],
        # )

        # Get the joint order
        self.jacobian_joint_order = []
        for i in range(self.chain.getNrOfSegments()):
            joint = self.chain.getSegment(i).getJoint()
            if joint.getType() != kdl.Joint.Fixed:
                self.jacobian_joint_order.append(joint.getName())
        self.JOINTS_ARM_I = [
            self.jacobian_joint_order.index(joint_name)
            for joint_name in self.JOINTS_ARM
        ]

        # Get the Jacobian solver
        self.jacobian_solver = kdl.ChainJntToJacSolver(self.chain)

        # Store the control gains
        # NOTE: The CMU implementation had controller gains of 0.2 on the
        # telescoping arm joints and 1.0 otherwise, but for this implementation
        # 1.0 everywhere seems to work fine.
        self.K = np.eye(len(self.jacobian_joint_order), dtype=np.float64)

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
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.node.get_logger().info("Waiting for the get joint states service...")
        get_joint_limits_client.wait_for_service()
        response = get_joint_limits_client.call(Trigger.Request())
        if not response.success:
            self.node.get_logger().error("Failed to get joint limits.")
            return False

        # Get the robot parameters
        robot_params = RobotParams().get_params()[1]
        speed_profile_str = speed_profile.value
        self.joint_pos_lim: Dict[str, Tuple[float, float]] = {}
        self.joint_vel_lim: Dict[str, Tuple[float, float]] = {}

        # Get the velocity limits from the robot's parameters
        for i, joint_name in enumerate(self.jacobian_joint_order):
            # Get the max velocity
            if joint_name == self.JOINT_BASE_TRANSLATION:
                max_vel = robot_params["base"]["motion"][speed_profile_str]["vel_m"]
            elif joint_name == self.JOINT_BASE_REVOLUTION:
                # From https://github.com/hello-robot/stretch_visual_servoing/blob/f99342/normalized_velocity_control.py#L21
                max_vel = 0.5  # rad/s
            elif joint_name == self.JOINT_ARM_LIFT:
                max_vel = robot_params["lift"]["motion"][speed_profile_str]["vel_m"]
            elif joint_name in self.JOINTS_ARM:
                continue
            elif joint_name == self.JOINT_WRIST_YAW:
                max_vel = robot_params["wrist_yaw"]["motion"][speed_profile_str]["vel"]
            elif joint_name == self.JOINT_WRIST_PITCH:
                max_vel = robot_params["wrist_pitch"]["motion"][speed_profile_str][
                    "vel"
                ]
            elif joint_name == self.JOINT_WRIST_ROLL:
                max_vel = robot_params["wrist_roll"]["motion"][speed_profile_str]["vel"]
            else:
                self.node.get_logger().warn(f"Unknown joint name: {joint_name}")
                max_vel = 0.0
            self.joint_vel_lim[joint_name] = (-max_vel, max_vel)
        combined_arm_max_vel = robot_params["arm"]["motion"][speed_profile_str]["vel_m"]
        self.joint_vel_lim[self.JOINT_COMBINED_ARM] = (
            -combined_arm_max_vel,
            combined_arm_max_vel,
        )

        with self.latest_joint_limits_lock:
            joint_pos_lim = self.latest_joint_limits
        if joint_pos_lim is None:
            self.node.get_logger().error("Failed to get joint limits.")
            return False

        for joint_name, (min_pos, max_pos) in joint_pos_lim.items():
            if joint_name in self.jacobian_joint_order:
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
            self.latest_joint_state = [
                msg.position[msg.name.index(joint_name)]
                if joint_name in msg.name
                else 0.0
                for joint_name in self.jacobian_joint_order
            ]

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
                if joint_name in self.jacobian_joint_order
            }

    async def move_to_end_effector_pose(
        self,
        goal: PoseStamped,
        articulated_joints: List[str],
        termination: TerminationCriteria,
        additional_joint_positions: Dict[str, float] = {},
        rate_hz: float = 10.0,
        timeout_secs: float = 10.0,
    ) -> bool:
        """
        Move the end-effector to the goal pose.

        Parameters
        ----------
        goal: The goal pose.
        articulated_joints: The articulated joints. This must be a subset of self.articulated_joints.
        termination: The termination criteria.
        additional_joint_positions: The joint names and positions here will be appended to the
            articulated joints when commanding arm motion.
        rate_hz: The rate in Hz at which to control the robot.
        timeout_secs: The timeout in seconds.

        Returns
        -------
        bool: True if the termination criteria was reached, else False.
        """
        # Check if the controller is initialized
        if not self.initialized:
            self.node.get_logger().error("Controller is not initialized.")
            return False

        # Start the timer
        start_time = self.node.get_clock().now()
        timeout = Duration(seconds=timeout_secs)

        # Check the articulated joints. Base and arm/wrist joints cannot co-exist.
        articulated_joints = [
            joint_name
            for joint_name in articulated_joints
            if joint_name in self.jacobian_joint_order
        ]
        if len(articulated_joints) == 0:
            self.node.get_logger().error("No articulated joints specified.")
            return False
        for joint_name in additional_joint_positions.keys():
            if joint_name in articulated_joints:
                self.node.get_logger().error(
                    f"Cannot pass a fixed position for articulated joint {joint_name}."
                )
                return False
        non_articulated_joints_mask = np.array(
            [
                joint_name not in articulated_joints
                for joint_name in self.jacobian_joint_order
            ]
        )
        move_base = False
        move_arm = False
        for joint_name in articulated_joints:
            if joint_name in self.JOINTS_BASE:
                move_base = True
            else:
                move_arm = True
        if move_base and move_arm:
            self.node.get_logger().error(
                "Cannot articulate base and arm/wrist joints at the same time."
            )
            return False

        # Convert to the appropriate control mode
        self.__set_control_mode(
            ControlMode.POSITION if move_arm else ControlMode.NAVIGATION,
            timeout=remaining_time(self.node.get_clock().now(), start_time, timeout),
        )

        # Convert the goal pose to the odom frame, so it stays stationary as the robot moves.
        try:
            goal_odom = self.tf_buffer.transform(
                goal,
                self.FRAME_ODOM,
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
                f"Failed to transform goal pose to odom frame: {e}"
            )
            return False

        # Command motion until the termination criteria is reached
        rate = self.node.create_rate(rate_hz)
        err = None
        vel = None
        while not self.__reached_termination(termination, err, vel):
            # Leave early if ROS shuts down or timeout is reached
            if not rclpy.ok() or remaining_time(
                self.node.get_clock().now(), start_time, timeout
            ) <= Duration(seconds=0.0):
                return False

            # Get the error between the end-effector pose and the goal pose
            ok, err = self.__get_err(
                goal_odom,
                remaining_time(self.node.get_clock().now(), start_time, timeout),
            )
            if not ok:
                return False
            self.node.get_logger().info(f"Error: {err}")

            # Get the current joint state
            with self.latest_joint_state_lock:
                q = np_to_joint_array(self.latest_joint_state)
            self.node.get_logger().info(
                f"Joint State: {list(zip(self.jacobian_joint_order, self.latest_joint_state))}"
            )

            # Get the Jacobian matrix for the chain
            J = kdl.Jacobian(len(self.jacobian_joint_order))
            self.jacobian_solver.JntToJac(q, J)
            J = jacobian_to_np(J)
            self.node.get_logger().info(f"Jacobian: {J}")

            # Mask the Jacobian matrix to only include the articulated joints
            J[:, non_articulated_joints_mask] = 0.0

            # Calculate the pseudo-inverse of the Jacobian
            J_pinv = np.linalg.pinv(J, rcond=1e-6)
            self.node.get_logger().info(f"Jacobian Pseudo-Inverse: {J_pinv}")

            # Re-mask the pseudo-inverse to address any numerical issues
            J_pinv[non_articulated_joints_mask, :] = 0.0

            # Calculate the joint velocities
            vel = self.K @ J_pinv @ err
            self.node.get_logger().info(
                f"Joint Velocities: {list(zip(self.jacobian_joint_order, vel))}"
            )

            # Execute the velocities
            self.__execute_velocities(
                vel, move_base, move_arm, additional_joint_positions
            )

            # # Sleep. Note we can't use ROS's rate.sleep as that blocks. However, as a result,
            # # It is likely that our control rate will actually be lower than rate_hz.
            # await asyncio.sleep(1.0 / rate_hz)
            # TODO: Verify this yields control back to the action correctly for sending feedback!
            rate.sleep()

        return True

    def __set_control_mode(self, control_mode: ControlMode, timeout: Duration) -> bool:
        """
        Set the control mode.

        Parameters
        ----------
        control_mode: The control mode.
        timeout: The timeout.

        Returns
        -------
        bool: True if the control mode was successfully set.
        """
        # Check if the control mode is already set
        if control_mode == self.control_mode:
            return True
        # Get the appropriate client
        if control_mode == ControlMode.POSITION:
            client = self.switch_to_position_client
        else:
            client = self.switch_to_navigation_client
        # Invoke the service
        ready = client.wait_for_service(timeout_sec=timeout.nanoseconds / 1.0e9)
        if not ready:
            self.node.get_logger().error(f"Service {client.srv_name} not available.")
            return False
        result = client.call(Trigger.Request())
        if not result.success:
            self.node.get_logger().error(
                f"Failed to switch to {control_mode.value} mode."
            )
            return False
        # Update the control mode
        self.control_mode = control_mode
        return True

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

        # TODO: Add different tolerances per-joint depending on the joint limits.
        # This is because for some small velocity commands, some joints will be able
        # to execute them but others will not.

        if err is None or vel is None:
            return False  # Always allow the first iteration
        if termination == TerminationCriteria.ZERO_VEL:
            retval = np.allclose(vel, 0.0, atol=1.0e-2)
            self.node.get_logger().info(f"Reached Termination {retval}")
            return retval
        elif termination == TerminationCriteria.ZERO_ERR:
            return np.allclose(err, 0.0, atol=1.0e-2)
        else:
            self.node.get_logger().error(f"Unknown termination criteria: {termination}")
            return True  # auto-terminate

    def __get_err(
        self,
        goal: PoseStamped,
        timeout: Duration,
    ) -> Tuple[bool, npt.NDArray[np.float64]]:
        """
        Get the error between the goal pose and the current end effector pose.
        Returns the error in base link frame.

        Parameters
        ----------
        goal: The goal end effector pose.
        timeout: The timeout.

        Returns
        -------
        bool: Whether the error was successfully calculated.
        npt.NDArray[np.float64]: The error in base link frame. The error is a 6D vector
            consisting of the translation and rotation errors.
        """

        # Get the current end effector pose in base frame
        # TODO: Technically, the timeout here is double. Fix that!
        try:
            ee_transform = self.tf_buffer.lookup_transform(
                self.FRAME_BASE_LINK,
                self.FRAME_END_EFFECTOR_LINK,
                Time(),
                timeout=timeout,
            )
        except (
            tf2.ConnectivityException,
            tf2.ExtrapolationException,
            tf2.InvalidArgumentException,
            tf2.LookupException,
            tf2.TimeoutException,
            tf2.TransformException,
        ) as error:
            self.node.get_logger().error(f"Failed to lookup transform: {error}")
            return False, np.zeros(6)

        # Get the goal end effector pose in base frame
        try:
            goal.header.stamp = Time()  # Get the most recent transform
            goal_base = self.tf_buffer.transform(
                goal, self.FRAME_BASE_LINK, timeout=timeout
            )
        except (
            tf2.ConnectivityException,
            tf2.ExtrapolationException,
            tf2.InvalidArgumentException,
            tf2.LookupException,
            tf2.TimeoutException,
            tf2.TransformException,
        ) as error:
            self.node.get_logger().error(f"Failed to transform goal pose: {error}")
            return False, np.zeros(6)

        # Get the error in base frame
        err = np.zeros(6, dtype=np.float64)
        err[:3] = np.array(
            [
                goal_base.pose.position.x - ee_transform.transform.translation.x,
                goal_base.pose.position.y - ee_transform.transform.translation.y,
                goal_base.pose.position.z - ee_transform.transform.translation.z,
            ]
        )
        ee_orientation = euler_from_quaternion(
            [
                ee_transform.transform.rotation.x,
                ee_transform.transform.rotation.y,
                ee_transform.transform.rotation.z,
                ee_transform.transform.rotation.w,
            ],
            axes="rzyx",  # https://wiki.ros.org/geometry2/RotationMethods#Fixed_Axis_vs_Euler_Angles
        )
        goal_orientation = euler_from_quaternion(
            [
                goal_base.pose.orientation.x,
                goal_base.pose.orientation.y,
                goal_base.pose.orientation.z,
                goal_base.pose.orientation.w,
            ],
            axes="rzyx",  # https://wiki.ros.org/geometry2/RotationMethods#Fixed_Axis_vs_Euler_Angles
        )
        err[3:] = np.array(
            [
                goal_orientation[2] - ee_orientation[2],  # roll
                goal_orientation[1] - ee_orientation[1],  # pitch
                goal_orientation[0] - ee_orientation[0],  # yaw
            ]
        )

        return True, err

    def __execute_velocities(
        self,
        joint_velocities: npt.NDArray[np.float64],
        move_base: bool,
        move_arm: bool,
        additional_joint_positions: Dict[str, float] = {},
    ) -> bool:
        """
        Send the velocities to the ROS2 controllers.

        Parameters
        ----------
        joint_velocities: The joint velocities.
        move_base: Whether to move the base.
        move_arm: Whether to move the arm.
        additional_joint_positions: The joint names and positions here will be appended to the
            articulated joints when commanding arm motion.

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
        clipped_velocities = np.zeros(len(self.jacobian_joint_order), dtype=np.float64)
        for i, vel in enumerate(joint_velocities):
            joint_name = self.jacobian_joint_order[i]
            if i not in self.JOINTS_ARM_I:
                min_vel, max_vel = self.joint_vel_lim[joint_name]
                clipped_velocities[i] = np.clip(vel, min_vel, max_vel)
        arm_joints_vel = joint_velocities[self.JOINTS_ARM_I]
        arm_vel = sum(joint_velocities[self.JOINTS_ARM_I])
        arm_min_vel, arm_max_vel = self.joint_vel_lim[self.JOINT_COMBINED_ARM]
        clipped_arm_vel = np.clip(arm_vel, arm_min_vel, arm_max_vel)
        clipped_velocities[self.JOINTS_ARM_I] = (
            arm_joints_vel / arm_vel * clipped_arm_vel
        )

        # Send base commands
        if move_base:
            base_vel = Twist()
            base_vel.linear.x = clipped_velocities[
                self.jacobian_joint_order.index(self.JOINT_BASE_TRANSLATION)
            ]
            base_vel.angular.z = clipped_velocities[
                self.jacobian_joint_order.index(self.JOINT_BASE_REVOLUTION)
            ]
            self.base_vel_pub.publish(base_vel)
            return True

        # Send arm commands
        duration = 0.5  # seconds, how long to forward-project velocities for
        with self.latest_joint_state_lock:
            latest_joint_state = self.latest_joint_state
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.joint_names = [
            joint_name
            for i, joint_name in enumerate(self.jacobian_joint_order)
            if (
                joint_name not in self.JOINTS_BASE
                and not np.isclose(clipped_velocities[i], 0.0, atol=5e-2)
            )
        ]
        arm_goal.trajectory.points = [JointTrajectoryPoint()]
        arm_goal.trajectory.points[0].positions = []
        arm_goal.trajectory.points[0].time_from_start = Duration(
            seconds=duration
        ).to_msg()
        for joint_name in arm_goal.trajectory.joint_names:
            pos = latest_joint_state[self.jacobian_joint_order.index(joint_name)]
            vel = clipped_velocities[self.jacobian_joint_order.index(joint_name)]
            arm_goal.trajectory.points[0].positions.append(pos + vel * duration)
        # Add the additional joint positions
        arm_goal.trajectory.joint_names.extend(additional_joint_positions.keys())
        arm_goal.trajectory.points[0].positions.extend(
            additional_joint_positions.values()
        )
        # Send the command
        self.arm_client_future = self.arm_client.send_goal_async(arm_goal)

        return True

    @property
    def articulable_joints(self) -> List[str]:
        """
        Get the articulable joints.

        Returns
        -------
        List[str]: The articulable joints.
        """
        if len(self.jacobian_joint_order) == 0:
            self.node.get_logger().error("Jacobian solver not loaded.")
            return []
        return self.jacobian_joint_order


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


def jacobian_to_np(J: kdl.Jacobian) -> npt.NDArray[np.float64]:
    """
    Convert a KDL Jacobian to a numpy array.

    Parameters
    ----------
    J: The KDL Jacobian.

    Returns
    -------
    npt.NDArray[np.float64]: The Jacobian as a numpy array.
    """
    return np.array([[J[i, j] for j in range(J.columns())] for i in range(J.rows())])


def np_to_joint_array(joint_state: npt.NDArray[np.float64]) -> kdl.JntArray:
    """
    Convert a numpy array to a KDL joint array.

    Parameters
    ----------
    joint_state: The numpy array.

    Returns
    -------
    kdl.JntArray: The KDL joint array.
    """
    q = kdl.JntArray(len(joint_state))
    for i, q_i in enumerate(joint_state):
        q[i] = q_i
    return q


def kdl_chain_fix_joints(chain: kdl.Chain, fixed_joints: List[str]) -> kdl.Chain:
    """
    Takes in a KDL chain, and returns a new chain with the specified joints fixed.

    Parameters
    ----------
    chain: The KDL chain.
    fixed_joints: The list of joint names to fix.

    Returns
    -------
    kdl.Chain: The fixed KDL chain.
    """

    fixed_chain = kdl.Chain()
    for i in range(chain.getNrOfSegments()):
        segment = chain.getSegment(i)
        joint = segment.getJoint()
        joint_type = (
            kdl.Joint.Fixed
            if (joint.getName() in fixed_joints or "Fixed" in joint.getTypeName())
            else kdl.Joint.RotAxis
            if "RotAxis" in joint.getTypeName()
            else kdl.Joint.TransAxis
        )
        if joint_type == kdl.Joint.Fixed:
            new_joint = kdl.Joint(
                name=joint.getName(),
                type=joint_type,
            )
        else:
            new_joint = kdl.Joint(
                name=joint.getName(),
                origin=joint.JointOrigin(),
                axis=joint.JointAxis(),
                type=joint_type,
            )
        try:
            fixed_chain.addSegment(
                kdl.Segment(
                    name=segment.getName(),
                    joint=new_joint,
                    f_tip=segment.getFrameToTip(),
                    I=segment.getInertia(),
                )
            )
        except Exception as e:
            raise Exception(
                f"Failed to add joint {joint.getName()} {joint.getType()} {joint.getTypeName()} {joint_type}: {e}"
            )

    return fixed_chain
