#!/usr/bin/env python3

# Standard Imports
import os
import pprint
import subprocess
import sys
import threading
from enum import Enum
from typing import Dict, List, Optional, Tuple

# Third-Party Imports
import message_filters
import numpy as np
import numpy.typing as npt
import pcl
import PyKDL as kdl
import rclpy
import ros2_numpy
import tf2_py as tf2
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import (
    Point,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    Vector3,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from std_msgs.msg import Header
from stretch_body.robot_params import RobotParams
from tf2_geometry_msgs import PointStamped, PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_about_axis
from trajectory_msgs.msg import JointTrajectoryPoint
from urdf import treeFromString

# Local Imports
from stretch_web_teleop.action import MoveToPregrasp


class MoveToPregraspState(Enum):
    """
    Move-to-pregrasp proceeds in the following sequential states:
      1. Moving/rotating the base to the target pose.
      2. Moving the arm & wrist to the target pose.

    TODO: Consider adding an initial state that stows the arm.
    """

    pass


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

    def __init__(self, image_params_file: str, tf_timeout_secs: float = 0.5):
        """
        Initialize the MoveToPregraspNode

        Parameters
        ----------
        image_params_file: The path to the YAML file configuring the video streams
            that are sent to the web app. This is necessary because this node has to undo
            the transforms before deprojecting the clicked pixel.
        tf_timeout_secs: The timeout in seconds for TF lookups.
        """
        super().__init__("move_to_pregrasp")

        # Load the video stream parameters
        with open(image_params_file, "r") as params:
            self.image_params = yaml.safe_load(params)

        # Load the robot parameters
        self.robot_params = RobotParams().get_params()[1]

        # Load the Jacobian solver
        self.jacobian_solver = None
        self.jacobian_joint_order: List[str] = []
        self.K: Optional[npt.NDArray[np.float32]] = None
        self.load_jacobian_solver()

        # Use masks to limit which joints we move and which cartesian dimensions we control
        # For starters, only control joint lift and z.
        articulated_joints = [
            "joint_lift",
            # "joint_wrist_yaw",
            "joint_wrist_roll",
        ]
        self.joint_mask = np.array(
            [
                joint_name in articulated_joints
                for joint_name in self.jacobian_joint_order
            ]
        )
        self.cartesian_mask = np.ones((6,), dtype=bool)

        # Initialize TF2
        self.tf_timeout = Duration(seconds=tf_timeout_secs)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

        # Subscribe to the robot's current joint state
        self.latest_joint_state_lock = threading.Lock()
        self.latest_joint_state = None
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Create a publisher and action client to command robot motion
        self.base_vel_pub = self.create_publisher(
            Twist,
            "/stretch/cmd_vel",
            qos_profile=1,
        )
        self.arm_client_future = None
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

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

    def load_jacobian_solver(self) -> bool:
        """
        Load the Jacobian solver for the robot.
        """
        # Get the path to the XACRO file
        # TODO: Get the URDF from a service, so this works on other robots other than the SE3
        stretch_description_dir = get_package_share_directory("stretch_description")
        urdf_file = os.path.join(
            stretch_description_dir,
            "urdf/stretch_description_SE3_eoa_wrist_dw3_tool_sg3.xacro",
        )
        # Get the URDF string
        urdf_string = subprocess.check_output(["xacro", urdf_file])
        # Load the URDF
        ok, robot = treeFromString(urdf_string)
        if not ok:
            self.get_logger().error(f"Failed to load the URDF at {urdf_file}")
            return False

        # Get the original chain
        orig_chain = robot.getChain(self.BASE_LINK, self.END_EFFECTOR_LINK)

        # Add dummy links and joints for base translation and revolution
        chain = kdl.Chain()
        chain.addSegment(
            kdl.Segment(
                name="link_base_translation",
                joint=kdl.Joint(
                    name="joint_base_translation",
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
                    name="joint_base_revolution",
                    origin=kdl.Vector(0, 0, 0),
                    axis=kdl.Vector(1, 0, 0),
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

        # Get the Jacobian solver
        self.jacobian_solver = kdl.ChainJntToJacSolver(self.chain)

        # Get the joint order
        self.jacobian_joint_order = []
        for i in range(self.chain.getNrOfSegments()):
            joint = self.chain.getSegment(i).getJoint()
            if joint.getType() != kdl.Joint.Fixed:
                self.jacobian_joint_order.append(joint.getName())

        # Store the control gains
        self.K = np.eye(len(self.jacobian_joint_order), dtype=np.float64)
        for i, joint_name in enumerate(self.jacobian_joint_order):
            if joint_name in [
                "joint_arm_l3",
                "joint_arm_l2",
                "joint_arm_l1",
                "joint_arm_l0",
            ]:
                self.K[i, i] = 0.2

        return True

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

    def joint_state_cb(self, msg: JointState) -> None:
        """
        Callback for the joint state subscriber. Save the latest joint state message.

        Parameters
        ----------
        msg: The joint state message.
        """
        with self.latest_joint_state_lock:
            self.latest_joint_state = [
                msg.position[msg.name.index(joint_name)]
                if joint_name in msg.name
                else 0.0  # TODO: Setting base translation and revolution to 0.0 seems suspect
                for joint_name in self.jacobian_joint_order
            ]

    def goal_callback(self, goal_request: MoveToPregrasp.Goal) -> GoalResponse:
        """
        Accept a goal if this action does not already have an active goal, else reject.

        Parameters
        ----------
        goal_request: The goal request message.
        """
        self.get_logger().info(f"Received request {goal_request}")

        # Reject the goal if the jacobian solver has not been loaded
        if self.jacobian_solver is None:
            self.get_logger().info(
                "Rejecting goal request since the Jacobian solver failed to load"
            )
            return GoalResponse.REJECT

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

        # TODO: Add a timeout

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

        # Get the goal end effector pose
        ok, goal_pose = self.get_goal_pose(
            x, y, z, pointcloud_msg.header, goal_handle.request
        )
        if not ok:
            self.get_logger().error("Failed to get goal pose")
            goal_handle.abort()
            return MoveToPregrasp.Result()
        self.get_logger().info(f"Goal Pose: {goal_pose}")

        # Keep moving the robot until the end effector is aligned with the clicked pixel
        rate = self.create_rate(10)
        joint_velocities = None
        while joint_velocities is None or not np.all(
            np.isclose(joint_velocities, 0.0, atol=5e-2)
        ):
            ok, err = self.get_err(goal_pose)
            if not ok:
                continue
            # TODO: test for cancellations and rclpy shutdown and such. And TF lookup errors.
            # TODO: publish feedback
            self.get_logger().info(f"Error: {err}")

            # Get the current joint state
            with self.latest_joint_state_lock:
                q = np_to_joint_array(self.latest_joint_state)
            self.get_logger().info(
                f"Joint State: {list(zip(self.jacobian_joint_order, self.latest_joint_state))}"
            )

            # Get the Jacobian matrix for the chain
            J = kdl.Jacobian(len(self.jacobian_joint_order))
            self.jacobian_solver.JntToJac(q, J)
            J = jacobian_to_np(J)

            # Apply the mask
            J[np.logical_not(self.cartesian_mask), :] = 0.0
            J[:, np.logical_not(self.joint_mask)] = 0.0

            # Calculate the pseudo-inverse of the Jacobian
            J_pinv = np.linalg.pinv(J)
            self.get_logger().info(f"Jacobian Pseudo-Inverse: {J_pinv}")

            # Calculate the joint velocities
            joint_velocities = self.K @ (J_pinv @ err)
            self.get_logger().info(
                f"Joint Velocities: {list(zip(self.jacobian_joint_order, joint_velocities))}"
            )

            # Execute the velocities
            self.execute_velocities(joint_velocities)

            rate.sleep()

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
        self, x: float, y: float, z: float, header: Header, request: MoveToPregrasp.Goal
    ) -> Tuple[bool, PoseStamped]:
        """
        Get the goal end effector pose.

        Parameters
        ----------
        x: The x-coordinate of the clicked point.
        y: The y-coordinate of the clicked point.
        z: The z-coordinate of the clicked point.
        header: The header of the pointcloud message.
        request: The goal request.

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

        # Determine how the robot should orient its gripper to align with the clicked pixel
        if (
            request.pregrasp_direction
            == MoveToPregrasp.Goal.PREGRASP_DIRECTION_HORIZONTAL
        ):
            horizontal_grasp = True
        elif (
            request.pregrasp_direction
            == MoveToPregrasp.Goal.PREGRASP_DIRECTION_VERTICAL
        ):
            self.get_logger().warn(
                "Vertical grasp not implemented yet. Defaulting to horizontal grasp."
            )
            horizontal_grasp = True
        else:  # auto
            self.get_logger().warn(
                "Auto grasp not implemented yet. Defaulting to horizontal grasp."
            )
            horizontal_grasp = True

        # Get the goal orientation in base frame
        try:
            self.get_logger().info(f"Pre1 {goal_position}")
            goal_position_base = self.tf_buffer.transform(
                goal_position, self.BASE_LINK, timeout=self.tf_timeout
            )
            self.get_logger().info(f"Post1 {goal_position_base}")
            if horizontal_grasp:
                # The goal orientation is (0,0,0,1) in base link frame, rotated
                # so +x points towards the clicked point
                theta = np.arctan2(
                    goal_position_base.point.y, goal_position_base.point.x
                )
                self.get_logger().info(f"Theta: {theta}")
                rot = quaternion_about_axis(theta, [0, 0, 1])
                self.get_logger().info(f"Rot: {rot}")
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

    def get_err(
        self,
        goal_pose: PoseStamped,
    ) -> Tuple[bool, npt.NDArray[np.float64]]:
        """
        Get the error between the goal pose and the current end effector pose.
        Returns the error in base link frame.

        Parameters
        ----------
        goal_pose: The goal end effector pose.

        Returns
        -------
        bool: Whether the error was successfully calculated.
        npt.NDArray[np.float64]: The error in base link frame. The error is a 6D vector
            consisting of the translation and rotation errors.
        """

        # Get the current end effector pose in base frame
        try:
            ee_transform = self.tf_buffer.lookup_transform(
                self.BASE_LINK,
                self.END_EFFECTOR_LINK,
                Time(),
                timeout=self.tf_timeout,
            )
        except (
            tf2.ConnectivityException,
            tf2.ExtrapolationException,
            tf2.InvalidArgumentException,
            tf2.LookupException,
            tf2.TimeoutException,
            tf2.TransformException,
        ) as error:
            self.get_logger().error(f"Failed to lookup transform: {error}")
            return False, np.zeros(6)

        # Get the goal end effector pose in base frame
        try:
            goal_pose.header.stamp = Time()  # Get the most recent transform
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
            self.get_logger().error(f"Failed to transform goal pose: {error}")
            return False, np.zeros(6)

        # Get the error in base frame
        err = np.zeros(6)
        err[:3] = np.array(
            [
                goal_pose_base.pose.position.x - ee_transform.transform.translation.x,
                goal_pose_base.pose.position.y - ee_transform.transform.translation.y,
                goal_pose_base.pose.position.z - ee_transform.transform.translation.z,
            ]
        )
        ee_orientation = euler_from_quaternion(
            [
                ee_transform.transform.rotation.x,
                ee_transform.transform.rotation.y,
                ee_transform.transform.rotation.z,
                ee_transform.transform.rotation.w,
            ]
        )
        goal_orientation = euler_from_quaternion(
            [
                goal_pose_base.pose.orientation.x,
                goal_pose_base.pose.orientation.y,
                goal_pose_base.pose.orientation.z,
                goal_pose_base.pose.orientation.w,
            ]
        )
        err[3:] = np.array(
            [
                goal_orientation[0] - ee_orientation[0],  # roll
                goal_orientation[1] - ee_orientation[1],  # pitch
                goal_orientation[2] - ee_orientation[2],  # yaw
            ]
        )

        return True, err

    def execute_velocities(self, joint_velocities: npt.NDArray[np.float64]) -> None:
        """
        Execute the joint velocities.

        Given the lack of a ROS2 velocity control interface for the arm, this function
        currently sends velocity commands to the base, and sends position commands to
        the arm as single-point "trajectories".

        Parameters
        ----------
        joint_velocities: The joint velocities.
        """
        # How long to forward-project the velocity to convert to positions
        duration = 1.0  # seconds

        # Clip the velocities
        clipped_velocities = self.clip_velocities(joint_velocities)

        # Only send arm commands if the previous arm command has finished
        send_arm_commands = (
            self.arm_client_future is None
        ) or self.arm_client_future.done()

        # TODO: Add joint limits

        # Get the current joint state
        if send_arm_commands:
            with self.latest_joint_state_lock:
                latest_joint_state = self.latest_joint_state

            # Create the action goal for the arm
            arm_goal = FollowJointTrajectory.Goal()
            arm_goal.trajectory.joint_names = [
                joint_name
                for i, joint_name in enumerate(self.jacobian_joint_order)
                if (
                    joint_name
                    not in ["joint_base_translation", "joint_base_revolution"]
                    and not np.isclose(joint_velocities[i], 0.0, atol=5e-2)
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

        # Create the base velocity message
        base_vel = Twist()
        base_vel.linear.x = clipped_velocities[
            self.jacobian_joint_order.index("joint_base_translation")
        ]
        base_vel.angular.z = clipped_velocities[
            self.jacobian_joint_order.index("joint_base_revolution")
        ]

        # Send the commands
        if send_arm_commands:
            self.arm_client_future = self.arm_client.send_goal_async(arm_goal)
        self.base_vel_pub.publish(base_vel)

    def clip_velocities(
        self, joint_velocities: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """
        Clip the joint velocities to the robot's velocity limits.

        Parameters
        ----------
        joint_velocities: The joint velocities.

        Returns
        -------
        npt.NDArray[np.float64]: The clipped joint velocities.
        """
        speed = "fast"  # options: slow, default, fast, max

        # Arm joints need to be treated separately since the max velocity is
        # for the entire arm, not individual joints.
        arm_joints = [
            "joint_arm_l3",
            "joint_arm_l2",
            "joint_arm_l1",
            "joint_arm_l0",
        ]
        arm_i = [
            self.jacobian_joint_order.index(joint_name) for joint_name in arm_joints
        ]

        # Clip the velocities
        clipped_vel = np.zeros(joint_velocities.shape, dtype=np.float64)
        for i, joint_name in enumerate(self.jacobian_joint_order):
            # Get the max velocity
            if joint_name == "joint_base_translation":
                max_vel = self.robot_params["base"]["motion"][speed]["vel_m"]
            elif joint_name == "joint_base_revolution":
                max_vel = 0.5  # rad/s
            elif joint_name == "joint_lift":
                max_vel = self.robot_params["lift"]["motion"][speed]["vel_m"]
            elif joint_name in arm_joints:
                # Clip arm joints separately, as then have to be treated as a vector
                continue
            elif joint_name == "joint_wrist_yaw":
                max_vel = self.robot_params["wrist_yaw"]["motion"][speed]["vel"]
            elif joint_name == "joint_wrist_pitch":
                max_vel = self.robot_params["wrist_pitch"]["motion"][speed]["vel"]
            elif joint_name == "joint_wrist_roll":
                max_vel = self.robot_params["wrist_roll"]["motion"][speed]["vel"]
            else:
                self.get_logger().warn(f"Unknown joint name: {joint_name}")
                max_vel = 0.0

            # Clip the velocity
            vel = joint_velocities[i]
            clipped_vel[i] = np.clip(vel, -max_vel, max_vel)
        # Clip arm joints separately, as then have to be treated as a vector
        arm_joints_vel = np.array(joint_velocities[arm_i])
        arm_speed = np.linalg.norm(arm_joints_vel, ord=1)
        arm_max_vel = self.robot_params["arm"]["motion"][speed]["vel_m"]
        if arm_speed > arm_max_vel:
            arm_joints_vel = arm_joints_vel / arm_speed * arm_max_vel
        clipped_vel[arm_i] = arm_joints_vel

        return clipped_vel


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    image_params_file = args[0]

    move_to_pregrasp = MoveToPregraspNode(image_params_file)
    move_to_pregrasp.get_logger().info("Initialized!")

    # Use a MultiThreadedExecutor so that subscriptions, actions, etc. can be
    # processed in parallel.
    executor = MultiThreadedExecutor(num_threads=4)

    rclpy.spin(move_to_pregrasp, executor=executor)

    move_to_pregrasp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
