#!/usr/bin/env python3

# Standard Imports
import os
import sys
import threading
import traceback
from typing import Dict, Generator, List, Optional, Tuple

# Third-Party Imports
import numpy as np
import numpy.typing as npt
import rclpy
import tf2_ros
import yaml
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped, Vector3
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Header
from tf2_geometry_msgs import PoseStamped
from tf_transformations import quaternion_about_axis, quaternion_multiply

# Local Imports
from stretch_web_teleop.action import MoveToPregrasp
from stretch_web_teleop_helpers.constants import (
    Frame,
    Joint,
    adjust_arm_lift_for_base_collision,
    get_pregrasp_wrist_configuration,
    get_stow_configuration,
)
from stretch_web_teleop_helpers.conversions import (
    deproject_pixel_to_pointcloud_point,
    depth_img_to_pointcloud,
    remaining_time,
    ros_msg_to_cv2_image,
    tf2_transform,
)
from stretch_web_teleop_helpers.move_to_pregrasp_state import MoveToPregraspState
from stretch_web_teleop_helpers.stretch_ik_control import (
    MotionGeneratorRetval,
    StretchIKControl,
)


class MoveToPregraspNode(Node):
    """
    The MoveToPregrasp node exposes an action server that takes in the
    (x, y) pixel coordinates of an operator's click on the Realsense
    camera feed. It then moves the robot so its end-effector is
    aligned with the clicked pixel, making it easy for the user to grasp
    the object the pixel is a part of.
    """

    # How far (m) from the object the end-effecor should move to.
    # We will use the first one that succeeds. The reason for multiple is in case
    # the object is close to the robot, and the robot can't actually get 0.1m away
    # from it.
    DISTANCES_TO_OBJECT = [0.1, 0.05, 0.025, 0.001]

    def __init__(
        self,
        image_params_file: str,
        tf_timeout_secs: float = 0.5,
        # This is high because the robot can take a while to align, and if the user
        # notices a lack of movement they can cancel the goal.
        action_timeout_secs: float = 60.0,
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
        self.static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.lift_offset: Optional[Tuple[float, float]] = None
        self.wrist_offset: Optional[Tuple[float, float]] = None

        # Create the inverse jacobian controller to execute motions
        urdf_abs_path = os.path.join(
            get_package_share_directory("stretch_web_teleop"),
            "urdf/stretch_base_rotation_ik.urdf",
        )
        self.controller = StretchIKControl(
            self,
            tf_buffer=self.tf_buffer,
            urdf_path=urdf_abs_path,
            static_transform_broadcaster=self.static_transform_broadcaster,
        )

        # Subscribe to the Realsense's RGB, pointcloud, and camera info feeds
        self.cv_bridge = CvBridge()
        self.latest_realsense_depth_lock = threading.Lock()
        self.latest_realsense_depth: Optional[CompressedImage] = None
        self.depth_image_subscriber = self.create_subscription(
            CompressedImage,
            "/camera/aligned_depth_to_color/image_raw/compressedDepth",
            self.realsense_depth_cb,
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.latest_realsense_info_lock = threading.Lock()
        self.latest_realsense_info: Optional[CameraInfo] = None
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            "/camera/aligned_depth_to_color/camera_info",
            self.realsense_info_cb,
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
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
            callback_group=ReentrantCallbackGroup(),
        )

        return True

    def realsense_depth_cb(
        self,
        depth_msg: CompressedImage,
    ) -> None:
        """
        Callback for the Realsense camera feed subscriber. Save the latest depth image.

        Parameters
        ----------
        depth_msg: The depth image message
        """
        with self.latest_realsense_depth_lock:
            self.latest_realsense_depth = depth_msg

    def realsense_info_cb(
        self,
        info_msg: CameraInfo,
    ) -> None:
        """
        Callback for the Realsense camera info subscriber. Save the latest camera info.

        Parameters
        ----------
        info_msg: The camera info message
        """
        with self.latest_realsense_info_lock:
            self.latest_realsense_info = info_msg

    def goal_callback(self, goal_request: MoveToPregrasp.Goal) -> GoalResponse:
        """
        Accept a goal if this action does not already have an active goal, else reject.

        Parameters
        ----------
        goal_request: The goal request message.
        """
        self.get_logger().info(f"Received request {goal_request}")

        # Reject the goal if no Realsense camera info has been received
        with self.latest_realsense_info_lock:
            if self.latest_realsense_info is None:
                self.get_logger().info(
                    "Rejecting goal request since no Realsense camera info message has been received yet"
                )
                return GoalResponse.REJECT

        # Reject the goal if no Realsense messages have been received yet
        with self.latest_realsense_depth_lock:
            if self.latest_realsense_depth is None:
                self.get_logger().info(
                    "Rejecting goal request since no Realsense depth message has been received yet"
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
            """
            Callback for when the error between the current end-effector pose and the goal
            end-effector pose is calculated. Send it as feedback.
            """
            self.get_logger().debug(f"Distance error: {err}")
            distance = np.linalg.norm(err[:3])
            if feedback.initial_distance_m < 0.0:
                feedback.initial_distance_m = distance
            else:
                feedback.remaining_distance_m = distance

        # Functions to cleanup the action
        terminate_motion_executors = False
        motion_executors: List[Generator[MotionGeneratorRetval, None, None]] = []

        def cleanup() -> None:
            """
            Clean up before returning from the action.
            """
            nonlocal terminate_motion_executors, motion_executors
            self.active_goal_request = None
            self.get_logger().debug("Setting termination flag to True")
            terminate_motion_executors = True
            # Execute the motion executors once more to process cancellation.
            if len(motion_executors) > 0:
                try:
                    for i, motion_executor in enumerate(motion_executors):
                        _ = next(motion_executor)
                except Exception:
                    self.get_logger().debug(traceback.format_exc())

        def action_error_callback(
            error_msg: str = "Goal failed",
            status: int = MoveToPregrasp.Result.STATUS_FAILURE,
        ) -> MoveToPregrasp.Result:
            """
            Callback for when the action encounters an error.
            """
            self.get_logger().error(error_msg)
            goal_handle.abort()
            cleanup()
            return MoveToPregrasp.Result(status=status)

        def action_success_callback(
            success_msg: str = "Goal succeeded",
        ) -> MoveToPregrasp.Result:
            """
            Callback for when the action succeeds.
            """
            self.get_logger().info(success_msg)
            goal_handle.succeed()
            cleanup()
            return MoveToPregrasp.Result(status=MoveToPregrasp.Result.STATUS_SUCCESS)

        def action_cancel_callback(
            cancel_msg: str = "Goal canceled",
        ) -> MoveToPregrasp.Result:
            """
            Callback for when the action is canceled.
            """
            self.get_logger().info(cancel_msg)
            goal_handle.canceled()
            cleanup()
            return MoveToPregrasp.Result(status=MoveToPregrasp.Result.STATUS_CANCELLED)

        # Get the clicked pixel
        x, y, z, header = self.get_clicked_pixel(goal_handle.request)

        # Determine how the robot should orient its gripper to align with the clicked pixel
        horizontal_grasp = self.get_grasp_orientation(goal_handle.request)

        # Get the goal end effector pose and verify it is reachable.
        reachable, goal_pose, base_rotation, ik_solution = self.get_goal_pose_and_ik(
            x, y, z, header, horizontal_grasp, publish_tf=True
        )
        if not reachable:
            return action_error_callback(
                f"Goal pose is not reachable {ik_solution}",
                MoveToPregrasp.Result.STATUS_GOAL_NOT_REACHABLE,
            )

        # Raise the arm lift if it is too low and the wrist would collide with the base
        adjust_arm_lift_for_base_collision(ik_solution, horizontal_grasp)

        # Get the states.
        states = self.get_states(horizontal_grasp, ik_solution)

        # Change the mode to navigation mode
        ok = self.controller.set_navigation_mode(
            timeout=remaining_time(
                self.get_clock().now(),
                start_time,
                self.action_timeout,
            )
        )
        if not ok:
            return action_error_callback(
                "Failed to set navigation mode",
                MoveToPregrasp.Result.STATUS_STRETCH_DRIVER_FAILURE,
            )

        # Execute the states
        state_i = 0
        rate = self.create_rate(5.0)
        while rclpy.ok():
            concurrent_states = states[state_i]
            self.get_logger().info(
                f"Executing States: {concurrent_states}", throttle_duration_sec=1.0
            )
            # Check if a cancel has been requested
            if goal_handle.is_cancel_requested:
                return action_cancel_callback("Goal canceled")
            # Check if the action has timed out
            if (self.get_clock().now() - start_time) > self.action_timeout:
                return action_error_callback(
                    "Goal timed out", MoveToPregrasp.Result.STATUS_TIMEOUT
                )

            # Move the robot
            if len(motion_executors) == 0:
                for state in concurrent_states:
                    motion_executor = state.get_motion_executor(
                        controller=self.controller,
                        goal_pose=goal_pose,
                        ik_solution=ik_solution,
                        horizontal_grasp=horizontal_grasp,
                        timeout_secs=remaining_time(
                            self.get_clock().now(),
                            start_time,
                            self.action_timeout,
                            return_secs=True,
                        ),
                        check_cancel=lambda: terminate_motion_executors,
                        err_callback=distance_callback,
                    )
                    if motion_executor is None:
                        return action_success_callback("Goal succeeded")
                    motion_executors.append(motion_executor)
            # Check if the robot is done moving
            else:
                try:
                    for i, motion_executor in enumerate(motion_executors):
                        retval = next(motion_executor)
                        if retval == MotionGeneratorRetval.SUCCESS:
                            motion_executors.pop(i)
                            break
                        elif retval == MotionGeneratorRetval.FAILURE:
                            raise Exception("Failed to move to goal pose")
                        else:  # CONTINUE
                            pass
                    if len(motion_executors) == 0:
                        if MoveToPregraspState.ROTATE_BASE in concurrent_states:
                            # If we just completed the rotate base motion, update the goal yaw
                            # so future IK is correct.
                            self.update_goal_orientation(
                                goal_pose,
                                get_pregrasp_wrist_configuration(horizontal_grasp),
                                publish_tf=True,
                            )
                        state_i += 1
                except Exception as e:
                    self.get_logger().error(traceback.format_exc())
                    return action_error_callback(
                        f"Error executing the motion generator: {e}",
                        MoveToPregrasp.Result.STATUS_FAILURE,
                    )

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
        return action_error_callback("Failed to execute MoveToPregrasp")

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

    def get_clicked_pixel(
        self, request: MoveToPregrasp.Goal
    ) -> Tuple[float, float, float, Header]:
        """
        Get the 3D coordinates of the clicked pixel in camera frame.

        Parameters
        ----------
        goal_handle: The goal handle.

        Returns
        -------
        Tuple[float, float, float, Header]: The clicked pixel, and the header of the depth message.
        """
        # Get the latest Realsense messages
        with self.latest_realsense_depth_lock:
            depth_msg = self.latest_realsense_depth
        with self.latest_realsense_info_lock:
            camera_info_msg = self.latest_realsense_info
        depth_image = ros_msg_to_cv2_image(depth_msg, self.cv_bridge)
        pointcloud = depth_img_to_pointcloud(
            depth_image,
            f_x=camera_info_msg.k[0],
            f_y=camera_info_msg.k[4],
            c_x=camera_info_msg.k[2],
            c_y=camera_info_msg.k[5],
        )  # N x 3 array

        # Undo any transformation that were applied to the raw camera image before sending it
        # to the web app
        raw_scaled_u, raw_scaled_v = (
            request.scaled_u,
            request.scaled_v,
        )
        if (
            "realsense" in self.image_params
            and "default" in self.image_params["realsense"]
        ):
            params = self.image_params["realsense"]["default"]
        else:
            params = None
        u, v = self.inverse_transform_pixel(
            raw_scaled_u, raw_scaled_v, params, camera_info_msg
        )
        self.get_logger().debug(
            f"Clicked pixel after inverse transform (camera frame): {(u, v)}"
        )

        # Deproject the clicked pixel to get the 3D coordinates of the clicked point
        x, y, z = deproject_pixel_to_pointcloud_point(
            u, v, pointcloud, np.array(camera_info_msg.p).reshape(3, 4)
        )
        self.get_logger().debug(
            f"Closest point to clicked pixel (camera frame): {(x, y, z)}"
        )

        return x, y, z, depth_msg.header

    def inverse_transform_pixel(
        self,
        scaled_u: int,
        scaled_v: int,
        params: Optional[Dict],
        camera_info_msg: CameraInfo,
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
        camera_info_msg: The camera info message.

        Returns
        -------
        Tuple[int, int]: The transformed pixel coordinates.
        """
        # Get the image dimensions. Note that (w, h) are dimensions of the raw image,
        # while (u, v) are coordinates on the transformed image.
        w, h = camera_info_msg.width, camera_info_msg.height

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
                    "Invalid rotate image value: options are ROTATE_90_CLOCKWISE, "
                    "ROTATE_180, or ROTATE_90_COUNTERCLOCKWISE"
                )

        return u, v

    def get_goal_pose_and_ik(
        self,
        x: float,
        y: float,
        z: float,
        header: Header,
        horizontal_grasp: bool,
        publish_tf: bool = False,
    ) -> Tuple[bool, PoseStamped, float, Dict[Joint, float]]:
        """
        Get the goal end effector pose and verify it is reachable.

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
        bool: Whether the goal pose was successfully calculated.
        PoseStamped: The goal end effector pose.
        float: The estimated base rotation to get to the goal pose.
        Dict[Joint, float]: The IK solution.
        """
        reachable = False
        goal_pose = PoseStamped()
        base_rotation = 0.0
        ik_solution: Dict[Joint, float] = {}
        for distance_to_object in self.DISTANCES_TO_OBJECT:
            ok, goal_pose, base_rotation = self.get_goal_pose(
                x,
                y,
                z,
                header,
                horizontal_grasp,
                publish_tf=True,
                distance_to_object=distance_to_object,
            )
            if not ok:
                continue
            self.get_logger().info(f"Goal Pose: {goal_pose}")

            # Verify the goal is reachable, seeded with the estimate base rotation.
            wrist_rotation = get_pregrasp_wrist_configuration(horizontal_grasp)
            joint_overrides = {}
            joint_overrides.update(wrist_rotation)
            if base_rotation is not None:
                # Seed IK with the estimated base rotation
                joint_overrides[Joint.BASE_ROTATION] = base_rotation
            reachable, ik_solution = self.controller.solve_ik(
                goal_pose, joint_position_overrides=joint_overrides
            )
            if not reachable:
                continue
            else:
                break
        return reachable, goal_pose, base_rotation, ik_solution

    def get_goal_pose(
        self,
        x: float,
        y: float,
        z: float,
        header: Header,
        horizontal_grasp: bool,
        publish_tf: bool = False,
        distance_to_object: float = 0.1,  # m
    ) -> Tuple[bool, PoseStamped, Optional[float]]:
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
        distance_to_object: The distance to move the end effector away from the object.

        Returns
        -------
        ok: Whether the goal pose was successfully calculated.
        PoseStamped: The goal end effector pose.
        Optional[float]: The estimated base rotation to get to the goal pose.
        """
        # Get the goal position in camera frame
        goal_pose = PoseStamped()
        goal_pose.header = header
        goal_pose.pose.position = Point(x=x, y=y, z=z)
        goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Convert to base frame
        ok, goal_pose_base = tf2_transform(
            self.tf_buffer, goal_pose, Frame.BASE_LINK.value, self.tf_timeout
        )
        if not ok:
            self.get_logger().error("Failed to transform goal pose to base link frame")
            return False, PoseStamped(), None

        # Get the goal orientation
        ok, theta, base_theta = self.__get_goal_yaw(
            goal_pose_base.pose.position.x,
            goal_pose_base.pose.position.y,
            horizontal_grasp,
            account_for_offsets=True,
        )
        if not ok:
            self.get_logger().error("Failed to get goal yaw")
            return False, PoseStamped(), None
        r = quaternion_about_axis(theta, [0, 0, 1])
        if not horizontal_grasp:
            # For vertical grasp, the goal orientation is also rotated +90deg around y
            r = quaternion_multiply(r, quaternion_about_axis(np.pi / 2, [0, 1, 0]))
        goal_pose_base.pose.orientation = Quaternion(x=r[0], y=r[1], z=r[2], w=r[3])

        # Adjust the goal position by the distance to the object
        if horizontal_grasp:
            xy = np.array(
                [goal_pose_base.pose.position.x, goal_pose_base.pose.position.y]
            )
            xy_dist = np.linalg.norm(xy)
            if xy_dist < distance_to_object:
                self.get_logger().error(
                    f"Clicked point is too close to the robot: {xy_dist} < {distance_to_object}"
                )
                return False, PoseStamped(), None
            xy = xy / xy_dist * (xy_dist - distance_to_object)
            goal_pose_base.pose.position.x, goal_pose_base.pose.position.y = xy
        else:
            goal_pose_base.pose.position.z += distance_to_object

        self.get_logger().debug(f"Goal pose in base link frame: {goal_pose_base}")

        # Convert the goal pose to the odom frame so it stays fixed even as the robot moves
        ok, goal_pose_odom = tf2_transform(
            self.tf_buffer, goal_pose_base, Frame.ODOM.value, self.tf_timeout
        )
        if not ok:
            self.get_logger().error("Failed to transform goal pose to odom frame")
            return False, PoseStamped(), None

        if publish_tf:
            self.broadcast_static_transform(goal_pose_odom, "goal")

        return True, goal_pose_odom, base_theta

    def __get_goal_yaw(
        self, x_g: float, y_g: float, horizontal_grasp: bool, account_for_offsets: bool
    ) -> Tuple[bool, float, Optional[float]]:
        """
        Gets the yaw angle of the goal pose. The below equations were derived
        by manual IK calculations on a projection of the robot onto the XY plane.
        Let theta be the angle the base has rotated, let L be the arm length,
        let (x_o, y_o) be the lift offset in base frame, and let (x_w, y_w) be
        the wrist offset in the lift frame. Then, we have the following transforms
        (base-to-world, lift-to-base, armEnd-to-lift):

        w_T_b = [[cos(theta)    -sin(theta)    0]
                 [sin(theta)     cos(theta)    0]
                 [0              0             1]]
        b_T_l = [[0   1  x_o]
                 [-1  0  y_o]
                 [0   0  1  ]]
        l_T_a = [[1  0  L]
                 [0  1  0]
                 [0  0  1]

        Our goal is to find (theta, L) such that:
        w_T_b * b_T_l * l_T_a [[x_w]   = [[x_g]
                               [y_w]      [y_g]
                               [1  ]]     [1  ]]

        The solutions to that are:

        theta = acos((x_o + y_w) / sqrt(x_g^2 + y_g^2)) + atan2(y_g, x_g) - (0 or pi)
        L = - (y_g)cos(theta) + (x_g)sin(theta) + y_o

        We use whichever theta results in a positive L. And we then convert it
        to lift rotation by subtracting pi/2.

        Note that in these calculations, we assume wrist yaw and roll will be 0
        in the final pre-grasp pose.

        Parameters
        ----------
        x_g: The x-coordinate of the goal pose, in base frame.
        y_g: The y-coordinate of the goal pose, in base frame.
        horizontal_grasp: Whether the goal pose should be horizontal.
        account_for_offsets: if True, do the above calculation. If false, just
            return the angle of the vector from the base to (x_g, y_g), which
            is quite close to the correct answer.

        Returns
        -------
        bool: Whether the goal yaw was successfully calculated.
        float: The goal yaw.
        Optional[float]: The estimated base rotation to get to the goal pose, if computed.
        """
        if not account_for_offsets:
            return True, np.arctan2(y_g, x_g), None

        # First, get the lift offset
        if self.lift_offset is None:
            T = self.controller.get_transform(
                parent_link=Frame.BASE_LINK, child_link=Frame.LIFT_LINK
            )
            self.lift_offset = (T[0, 3], T[1, 3])
        x_o, y_o = self.lift_offset

        # Next, get the wrist offset
        if self.wrist_offset is None:
            # First, get the transform from the base link to the end of the arm
            T = self.controller.get_transform(
                parent_link=Frame.L0_LINK,
                child_link=Frame.END_EFFECTOR_LINK,
                joint_overrides=get_pregrasp_wrist_configuration(horizontal_grasp),
            )
            self.get_logger().debug(f"Transform from L0 to wrist pitch: {T}")
            # In the manual IK calculations, what I call +x and +y is what
            # the actual TF tree has as +z and +x, respectively.
            self.wrist_offset = (T[2, 3], T[0, 3])
        x_w, y_w = self.wrist_offset

        self.get_logger().debug(f"Lift offset: {(x_o, y_o)}")
        self.get_logger().debug(f"Wrist offset: {(x_w, y_w)}")
        self.get_logger().debug(f"Goal pose in base frame: {(x_g, y_g)}")

        # Get the possible thetas
        theta = np.arccos((x_o + y_w) / np.sqrt(x_g**2 + y_g**2)) + np.arctan2(
            y_g, x_g
        )
        thetas = [theta, theta - np.pi]

        # Get the corresponding Ls
        Ls = [
            -y_g * np.cos(theta) + x_g * np.sin(theta) + y_o - x_w for theta in thetas
        ]
        self.get_logger().debug(f"Possible thetas: {thetas}, Ls: {Ls}")
        self.get_logger().debug(f"Earlier theta: {np.arctan2(y_g, x_g)}")

        for i in range(2):
            if Ls[i] > 0:
                return True, thetas[i] - np.pi / 2, thetas[i]
        return False, 0.0, None

    def update_goal_orientation(
        self,
        goal_pose: PoseStamped,
        wrist_configuration: Dict[Joint, float],
        publish_tf: bool = False,
    ) -> bool:
        """
        Run FK with the specified wrist configuration to get the goal orientation in base
        frame, then convert it to odom frame and update the orientation of the goal pose.

        Parameters
        ----------
        goal_pose: The old goal pose.
        wrist_configuration: The wrist configuration.
        publish_tf: Whether to publish the goal pose as a TF frame.

        Returns
        -------
        bool: Whether it succeeded.
        """
        # Run FK
        ee_pos, ee_quat = self.controller.solve_fk(joint_overrides=wrist_configuration)
        ee_pose = PoseStamped()
        ee_pose.header.frame_id = Frame.BASE_LINK.value
        ee_pose.pose.position = Point(x=ee_pos[0], y=ee_pos[1], z=ee_pos[2])
        ee_pose.pose.orientation = Quaternion(
            x=ee_quat[0], y=ee_quat[1], z=ee_quat[2], w=ee_quat[3]
        )

        # Convert it to odom frame
        ok, ee_pose_odom = tf2_transform(
            self.tf_buffer, ee_pose, Frame.ODOM.value, self.tf_timeout
        )
        if not ok:
            self.get_logger().error(
                "Failed to transform end effector pose to the odom frame"
            )
            return False

        # Update the orientation of the goal pose
        goal_pose.pose.orientation = ee_pose_odom.pose.orientation

        if publish_tf:
            self.broadcast_static_transform(goal_pose, "goal")

        return True

    def get_states(
        self, horizontal_grasp: bool, ik_solution: Dict[Joint, float]
    ) -> List[List[MoveToPregraspState]]:
        """
        Get the states to execute to move the robot to the goal pose.

        Parameters
        ----------
        horizontal_grasp: Whether the goal pose should be horizontal.
        ik_solution: The IK solution.

        Returns
        -------
        List[MoveToPregraspState]: The states to execute.
        """
        # Get the current joints
        init_joints = self.controller.get_current_joints()
        # Get the arm lift/length when (partially) stowed
        arm_lift_when_stowed = get_stow_configuration([Joint.ARM_LIFT])[Joint.ARM_LIFT]
        arm_length_when_partially_stowed = get_stow_configuration(
            [Joint.ARM_L0], partial=True
        )[Joint.ARM_L0]

        # Get the states. If the robot is in a vertical grasp position and the arm
        # needs to descend, lengthn the arm before deploying the wrist.
        states = MoveToPregraspState.get_state_machine(
            horizontal_grasp=horizontal_grasp,
            init_lift_near_base=init_joints[Joint.ARM_LIFT] < arm_lift_when_stowed,
            goal_lift_near_base=ik_solution[Joint.ARM_LIFT] < arm_lift_when_stowed,
            init_length_near_mast=init_joints[Joint.ARM_L0]
            < arm_length_when_partially_stowed,
        )

        # Ensure the terminal state is the last state
        if not (len(states[-1]) == 1 and MoveToPregraspState.TERMINAL in states[-1]):
            self.get_logger().error(
                "Terminal state is not the last state. Adding it in."
            )
            states.append([MoveToPregraspState.TERMINAL])
        self.get_logger().debug(f"All States: {states}")

        return states

    def broadcast_static_transform(
        self, pose: PoseStamped, child_frame_id: str
    ) -> None:
        """
        Broadcast a static transform from the base frame to the goal pose.

        Parameters
        ----------
        pose: The goal pose.
        child_frame_id: The child frame id.
        """
        self.static_transform_broadcaster.sendTransform(
            TransformStamped(
                header=pose.header,
                child_frame_id=child_frame_id,
                transform=Transform(
                    translation=Vector3(
                        x=pose.pose.position.x,
                        y=pose.pose.position.y,
                        z=pose.pose.position.z,
                    ),
                    rotation=pose.pose.orientation,
                ),
            )
        )


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
