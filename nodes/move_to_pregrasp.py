#!/usr/bin/env python3

# Standard Imports
import sys
import threading
from typing import Dict, List, Optional, Tuple

# Third-party Imports
import message_filters
import numpy as np
import pcl
import rclpy
import ros2_numpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

# Local Imports
from stretch_web_teleop.action import MoveToPregrasp


class MoveToPregraspNode(Node):
    """
    The MoveToPregrasp node exposes an action server that takes in the
    (x, y) pixel coordinates of an operator's click on the Realsense
    camera feed. It then moves the robot so its end-effector is
    aligned with the clicked pixel, making it easy for the user to grasp
    the object the pixel is a part of.
    """

    def __init__(self, image_params_file: str):
        """
        Initialize the MoveToPregraspNode

        Parameters
        ----------
        image_params_file: The path to the YAML file configuring the video streams
            that are sent to the web app. This is necessary because this node has to undo
            the transforms before deprojecting the clicked pixel.
        """
        super().__init__("move_to_pregrasp")

        # Load the video stream parameters
        with open(image_params_file, "r") as params:
            self.image_params = yaml.safe_load(params)

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
            horizontal_grasp = True
        else:  # auto
            self.get_logger().warn(
                "Auto grasp not implemented yet. Defaulting to horizontal grasp."
            )
            horizontal_grasp = True

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

        # Deproject the clicked pixel to get the 3D coordinates of the clicked point
        with self.latest_realsense_msgs_lock:
            rgb_msg, pointcloud_msg = self.latest_realsense_msgs
        pointcloud = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(
            pointcloud_msg
        )  # N x 3 array
        ray_dir = np.linalg.pinv(self.p)[:3, :] @ np.array(
            [u, v, 1]
        )  # The direction of the pixel from the camera
        ray_dir /= np.linalg.norm(ray_dir)
        # Find the pointcloud point that is closest to the ray
        p, r = pointcloud, ray_dir
        closest_point_idx = np.argmin(
            np.linalg.norm(
                p - np.multiply((p @ r).reshape((-1, 1)), r.reshape((1, 3))), axis=1
            )
        )
        self.get_logger().info(
            f"Closest point to clicked pixel (camera frame): {p[closest_point_idx]}"
        )

        goal_handle.succeed()
        result = MoveToPregrasp.Result()
        self.active_goal_request = None
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
                # An image is guarenteed to exist by the precondition of accepting the goal
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


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    image_params_file = args[0]

    move_to_pregrasp = MoveToPregraspNode(image_params_file)
    move_to_pregrasp.get_logger().info("Initialized!")

    # Use a MultiThreadedExecutor so that subscriptions, actions, etc. can be
    # processed in parallel.
    executor = MultiThreadedExecutor(num_threads=4)

    rclpy.spin(move_to_pregrasp, executor=executor)


if __name__ == "__main__":
    main(sys.argv[1:])
