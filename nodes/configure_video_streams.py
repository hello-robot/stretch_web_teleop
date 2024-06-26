#!/usr/bin/env python3

import math
import sys
import threading
from typing import Dict, Union

import cv2
import numpy as np
import numpy.typing as npt
import pcl
import PyKDL  # TODO: This can be removed, as it is only used to perform a transformation that can be done with numpy
import rclpy
import ros2_numpy
import tf2_py as tf2
import tf2_ros
import yaml
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState, PointCloud2
from std_srvs.srv import SetBool

from stretch_web_teleop_helpers.conversions import (
    cv2_image_to_ros_msg,
    deproject_pixel_to_point,
    depth_img_to_pointcloud,
    ros_msg_to_cv2_image,
)

# TODO: Add docstrings to this file.


class ConfigureVideoStreams(Node):
    BACKGROUND_COLOR = (200, 200, 200)
    REALSENSE_DEPTH_AR_COLOR = (0, 191, 255)
    REALSENSE_DEPTH_AR_ALPHA = 0.6
    GRIPPER_DEPTH_AR_COLOR = (255, 0, 191)
    GRIPPER_DEPTH_AR_ALPHA = 0.6

    def __init__(
        self,
        params_file,
        has_beta_teleop_kit,
        use_overhead: bool = True,
        use_realsense: bool = True,
        use_gripper: bool = True,
        # Even with the aligned depth compression parameter set to 1 (fastest),
        # we are seeing that the aligned depth data is received 2-4x later than
        # pointclouds (pointclouds are sub-1s between the realsense header stamp
        # and when we receive it, whereas aligned depth images are up to 0.4s)
        use_pointcloud: bool = True,
        use_compressed_image: bool = True,
        verbose: bool = False,
    ):
        """
        Initialize the ConfigureVideoStreams class.

        Parameters
        ----------
        node: The ROS node that this class is a part of.
        params_file: The path to the YAML file containing the image parameters.
        use_overhead: If True, subscribe to the overhead camera image messages.
        use_realsense: If True, subscribe to the realsense camera image messages.
        use_gripper: If True, subscribe to the gripper camera image messages.
        has_beta_teleop_kit: Whether the robot has the beta teleop kit.
        use_pointcloud: If True, subscribe to the raw pointcloud message. If False,
            subscribe to the aligned depth image message.
        use_compressed_image: If True, subscribe to the compressed image messages.
            If False, subscribe to the raw image messages.
        verbose: If True, print additional log messages.
        """
        super().__init__("configure_video_streams")

        with open(params_file, "r") as params:
            self.image_params = yaml.safe_load(params)
        self.verbose = verbose

        # These are parameters the web app uses to determine which features to
        # enabled. They are not used in this node itself.
        self.declare_parameter("has_beta_teleop_kit", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("stretch_tool", rclpy.Parameter.Type.STRING)

        # Subscribe to the TF camera feeds to project camera points into base frame.
        if use_realsense:
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=12))
            self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Loaded params for each video stream
        if use_overhead:
            self.overhead_params = (
                self.image_params["overhead"]
                if "overhead" in self.image_params
                else None
            )
            self.overhead_images: Dict[str, npt.NDArray] = {}
            self.overhead_camera_rgb_image = None
        if use_realsense:
            self.realsense_params = (
                self.image_params["realsense"]
                if "realsense" in self.image_params
                else None
            )
            self.realsense_images: Dict[str, npt.NDArray] = {}
            self.realsense_rgb_image = None
        if use_gripper:
            self.gripper_params = (
                self.image_params["gripper"] if "gripper" in self.image_params else None
            )
            self.expanded_gripper_params = (
                self.image_params["expandedGripper"]
                if "expandedGripper" in self.image_params
                else None
            )
            self.gripper_images: Dict[str, npt.NDArray] = {}
            self.gripper_camera_rgb_image = None

        self.cv_bridge = CvBridge()
        self.aruco_detector = None
        # https://github.com/hello-robot/stretchpy/blob/feature/aruco_marker_detection/docs/arucos.md#known-markers
        self.gripper_aruco_ids = {
            "finger_left": 200,
            "finger_right": 201,
        }

        # Stores the camera projection matrix
        if use_realsense:
            self.realsense_P = None
        if use_gripper:
            self.gripper_P = None

        # Compressed Image publishers
        if use_overhead:
            self.publisher_overhead_cmp = self.create_publisher(
                CompressedImage, "/navigation_camera/image_raw/rotated/compressed", 15
            )
        if use_realsense:
            self.publisher_realsense_cmp = self.create_publisher(
                CompressedImage,
                "/camera/color/image_raw/rotated/compressed",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
        if use_gripper:
            self.publisher_gripper_cmp = self.create_publisher(
                CompressedImage,
                "/gripper_camera/image_raw/cropped/compressed",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

        # Subscribers
        if use_overhead:
            self.overhead_camera_rgb_subscriber = self.create_subscription(
                Image,  # usb_cam doesn't output compressed images
                "/navigation_camera/image_raw",
                self.navigation_camera_cb,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        if use_gripper:
            if has_beta_teleop_kit:
                self.gripper_camera_rgb_subscriber = self.create_subscription(
                    Image,
                    "/gripper_camera/image_raw",
                    self.gripper_camera_cb,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )
            else:
                # Subscribe to the compressed image topic
                self.gripper_camera_rgb_subscriber = self.create_subscription(
                    CompressedImage if use_compressed_image else Image,
                    "/gripper_camera/image_raw"
                    + ("/compressed" if use_compressed_image else ""),
                    self.gripper_realsense_rgb_cb,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )

                # Services for expanding the gripper image
                self.expanded_gripper_service = self.create_service(
                    SetBool, "expanded_gripper", self.expanded_gripper_callback
                )
                self.expanded_gripper = False

                # Subscribe to the depth image topic
                self.latest_gripper_realsense_depth_image = None
                self.latest_gripper_realsense_depth_image_lock = threading.Lock()
                if use_pointcloud:
                    self.gripper_depth_subscriber = self.create_subscription(
                        PointCloud2,
                        "/gripper_camera/depth/color/points",
                        self.gripper_realsense_depth_cb,
                        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                        callback_group=MutuallyExclusiveCallbackGroup(),
                    )
                else:
                    self.gripper_depth_subscriber = self.create_subscription(
                        CompressedImage if use_compressed_image else Image,
                        "/gripper_camera/aligned_depth_to_color/image_raw"
                        + ("/compressedDepth" if use_compressed_image else ""),
                        self.gripper_realsense_depth_cb,
                        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                        callback_group=MutuallyExclusiveCallbackGroup(),
                    )
                self.gripper_camera_info_subscriber = self.create_subscription(
                    CameraInfo,
                    "/gripper_camera/aligned_depth_to_color/camera_info",
                    self.gripper_camera_info_cb,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )

        if use_gripper:
            self.joint_state_subscription = self.create_subscription(
                JointState,
                "/stretch/joint_states",
                self.joint_state_cb,
                1,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

        if use_realsense:
            self.camera_rgb_subscriber = self.create_subscription(
                CompressedImage if use_compressed_image else Image,
                "/camera/color/image_raw"
                + ("/compressed" if use_compressed_image else ""),
                self.realsense_rgb_cb,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            self.latest_realsense_depth_image = None
            self.latest_realsense_depth_image_lock = threading.Lock()
            if use_pointcloud:
                self.depth_subscriber = self.create_subscription(
                    PointCloud2,
                    "/camera/depth/color/points",
                    self.realsense_depth_cb,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )
            else:
                self.depth_subscriber = self.create_subscription(
                    CompressedImage if use_compressed_image else Image,
                    "/camera/aligned_depth_to_color/image_raw"
                    + ("/compressedDepth" if use_compressed_image else ""),
                    self.realsense_depth_cb,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )
            self.camera_info_subscriber = self.create_subscription(
                CameraInfo,
                "/camera/aligned_depth_to_color/camera_info",
                self.realsense_camera_info_cb,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

        # Default image perspectives
        if use_overhead:
            self.overhead_camera_perspective = (
                "fixed" if has_beta_teleop_kit else "wide_angle_cam"
            )
        if use_realsense:
            self.realsense_camera_perspective = "default"
        if use_gripper:
            self.gripper_camera_perspective = (
                "default" if has_beta_teleop_kit else "d405"
            )

        # Services for enabling the depth AR overlay on the realsense stream
        if use_realsense:
            self.realsense_depth_ar_service = self.create_service(
                SetBool, "realsense_depth_ar", self.realsense_depth_ar_callback
            )
            self.realsense_depth_ar = False
        if use_gripper:
            self.gripper_depth_ar_service = self.create_service(
                SetBool, "gripper_depth_ar", self.gripper_depth_ar_callback
            )
            self.gripper_depth_ar = False

        self.pcl_cloud_filtered = None
        self.roll_value = 0.0

    # https://github.com/ros/geometry2/blob/noetic-devel/tf2_sensor_msgs/src/tf2_sensor_msgs/tf2_sensor_msgs.py#L44
    def transform_to_kdl(self, t):
        return PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ),
            PyKDL.Vector(
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ),
        )

    # https://github.com/ros/geometry2/blob/noetic-devel/tf2_sensor_msgs/src/tf2_sensor_msgs/tf2_sensor_msgs.py#L52
    def do_transform_cloud(self, cloud, transform):
        t_kdl = self.transform_to_kdl(transform)
        points_out = []
        points = cloud.to_array()
        for p_in in points:
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0], p_out[1], p_out[2]])
        return np.array(points_out)

    def realsense_camera_info_cb(self, msg):
        self.realsense_P = np.array(msg.p).reshape(3, 4)
        # self.camera_info_subscriber.destroy()

    def gripper_camera_info_cb(self, msg):
        self.gripper_P = np.array(msg.p).reshape(3, 4)
        # self.camera_info_subscriber.destroy()

    def overlay_realsense_depth_ar(
        self,
        depth_msg: Union[CompressedImage, Image, PointCloud2],
        img: npt.NDArray[np.uint8],
    ):
        # Only keep points that are within 0.01m to 1.5m from the camera
        if self.realsense_P is None:
            self.get_logger().warn(
                "Camera projection matrix is not available. Skipping point cloud processing."
            )
            return img

        if isinstance(depth_msg, (CompressedImage, Image)):
            depth_image = ros_msg_to_cv2_image(depth_msg, self.cv_bridge)
            pc_in_camera = depth_img_to_pointcloud(
                depth_image,
                f_x=self.realsense_P[0, 0],
                f_y=self.realsense_P[1, 1],
                c_x=self.realsense_P[0, 2],
                c_y=self.realsense_P[1, 2],
            )
        else:
            pc_in_camera = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(depth_msg)
        pcl_cloud = pcl.PointCloud(np.array(pc_in_camera, dtype=np.float32))
        passthrough = pcl_cloud.make_passthrough_filter()
        passthrough.set_filter_field_name("z")
        passthrough.set_filter_limits(0.01, 1.5)
        self.pcl_cloud_filtered = passthrough.filter()

        # Get transform
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_color_optical_frame",
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except (
            tf2.ConnectivityException,
            tf2.ExtrapolationException,
            tf2.InvalidArgumentException,
            tf2.LookupException,
            tf2.TimeoutException,
            tf2.TransformException,
        ) as error:
            self.get_logger().warn(
                "Could not find the transform between frames base_link and "
                f"camera_color_optical_frame. Error: {error}"
            )
            return img

        if not self.pcl_cloud_filtered:
            return img
        if self.pcl_cloud_filtered.to_array().size == 0:
            return img

        # Transform point cloud to base link
        pc_in_base_link = self.do_transform_cloud(self.pcl_cloud_filtered, transform)

        # Only keep points that are between 0.25m and 1m from the base in the XY plane
        dist = np.sqrt(
            np.power(pc_in_base_link[:, 0], 2) + np.power(pc_in_base_link[:, 1], 2)
        )
        filtered_indices = np.where((dist > 0.25) & (dist < 1))[0]

        # Get filtered points in camera frame
        pts_in_range = self.pcl_cloud_filtered.to_array()[filtered_indices, :]

        # Convert to homogeneous coordinates
        pts_in_range = np.hstack((pts_in_range, np.ones((pts_in_range.shape[0], 1))))

        # Change color of pixels in robot's reach
        if img is not None:
            # Get pixel coordinates
            coords = np.matmul(self.realsense_P, np.transpose(pts_in_range))  # 3 x N
            u_idx = (coords[0, :] / coords[2, :]).astype(int)
            v_idx = (coords[1, :] / coords[2, :]).astype(int)
            uv = np.vstack((u_idx, v_idx)).T  # N x 2
            uv_dedup = np.unique(uv, axis=0)
            in_bounds_idx = np.where(
                (uv_dedup[:, 0] >= 0)
                & (uv_dedup[:, 0] < img.shape[1])
                & (uv_dedup[:, 1] >= 0)
                & (uv_dedup[:, 1] < img.shape[0])
            )
            u_mask = uv_dedup[in_bounds_idx][:, 0]
            v_mask = uv_dedup[in_bounds_idx][:, 1]
            # Overlay the pixels in the robot's reach
            img = img.astype(np.float32)
            img[v_mask, u_mask, :] *= 1.0 - self.REALSENSE_DEPTH_AR_ALPHA
            img[v_mask, u_mask, :] += self.REALSENSE_DEPTH_AR_ALPHA * np.array(
                self.REALSENSE_DEPTH_AR_COLOR
            )
            img = img.astype(np.uint8)
        return img

    def realsense_depth_ar_callback(self, req, res):
        self.get_logger().info(f"Realsense depth AR service: {req.data}")
        self.realsense_depth_ar = req.data
        res.success = True
        return res

    def gripper_depth_ar_callback(self, req, res):
        self.get_logger().info(f"Gripper depth AR service: {req.data}")
        self.gripper_depth_ar = req.data
        res.success = True
        return res

    def expanded_gripper_callback(self, req, res):
        self.get_logger().info(f"Expanded gripper service: {req.data}")
        self.expanded_gripper = req.data
        res.success = True
        return res

    def crop_image(self, image, params):
        if params["x_min"] is None:
            raise ValueError("Crop x_min is not defined!")
        if params["x_max"] is None:
            raise ValueError("Crop x_max is not defined!")
        if params["y_min"] is None:
            raise ValueError("Crop y_min is not defined!")
        if params["y_max"] is None:
            raise ValueError("Crop y_max is not defined!")

        x_min = params["x_min"]
        x_max = params["x_max"]
        y_min = params["y_min"]
        y_max = params["y_max"]
        width = x_max - x_min
        height = y_max - y_min

        # It is possible that the "crop" expands the image beyond its original dimensions.
        # Hence, we create a new image and fill it with a constant value for the background.
        background_color = (
            self.BACKGROUND_COLOR
            if image.shape[-1] == 3
            else (*self.BACKGROUND_COLOR, 255)
        )
        cropped_image = (
            np.repeat(background_color, width * height, axis=0)
            .reshape(height, width, image.shape[-1])
            .astype(np.uint8)
        )

        # x and y are swapped, since the first index is the rows (y) and the second index is the columns (x)
        cropped_image[
            max(-y_min, 0) : min(height - (y_max - image.shape[0]), height),
            max(-x_min, 0) : min(width - (x_max - image.shape[1]), width),
        ] = image[
            max(y_min, 0) : min(y_max, image.shape[0]),
            max(x_min, 0) : min(x_max, image.shape[1]),
        ]
        return cropped_image

    # https://stackoverflow.com/questions/44865023/how-can-i-create-a-circular-mask-for-a-numpy-array
    def create_circular_mask(self, h, w, center=None, radius=None):
        if center is None:  # use the middle of the image
            center = (int(w / 2), int(h / 2))
        if (
            radius is None
        ):  # use the smallest distance between the center and image walls
            radius = min(center[0], center[1], w - center[0], h - center[1])

        Y, X = np.ogrid[:h, :w]
        dist_from_center = np.sqrt((X - center[0]) ** 2 + (Y - center[1]) ** 2)

        mask = dist_from_center <= radius
        return mask

    def mask_image(self, image, params):
        if params["width"] is None:
            raise ValueError("Mask width is not defined!")
        if params["height"] is None:
            raise ValueError("Mask height is not defined!")

        w = params["width"]
        h = params["height"]
        center = (
            (params["center"]["x"], params["center"]["y"]) if params["center"] else None
        )
        radius = params["radius"]

        mask = self.create_circular_mask(h, w, center, radius)
        img = image.copy()
        background_color = (
            self.BACKGROUND_COLOR
            if image.shape[-1] == 3
            else (*self.BACKGROUND_COLOR, 255)
        )
        img[~mask] = background_color
        return img

    def rotate_image(self, image, value):
        if value == "ROTATE_90_CLOCKWISE":
            return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif value == "ROTATE_180":
            return cv2.rotate(image, cv2.ROTATE_180)
        elif value == "ROTATE_90_COUNTERCLOCKWISE":
            return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            raise ValueError(
                "Invalid rotate image value: options are ROTATE_90_CLOCKWISE, ROTATE_180, or ROTATE_90_COUNTERCLOCKWISE"
            )

    def configure_images(self, rgb_image, params):
        color_transform = (
            cv2.COLOR_BGR2RGB if rgb_image.shape[-1] == 3 else cv2.COLOR_BGRA2RGBA
        )
        rgb_image = cv2.cvtColor(rgb_image, color_transform)
        if params:
            if params["crop"]:
                rgb_image = self.crop_image(rgb_image, params["crop"])
            if params["mask"]:
                rgb_image = self.mask_image(rgb_image, params["mask"])
            if params["rotate"]:
                rgb_image = self.rotate_image(rgb_image, params["rotate"])
        return rgb_image

    def realsense_depth_cb(
        self,
        depth_msg: Union[CompressedImage, Image, PointCloud2],
    ):
        if self.verbose:
            start_time = self.get_clock().now()
            lag = (
                start_time - Time.from_msg(depth_msg.header.stamp)
            ).nanoseconds / 1.0e9
            self.get_logger().info(
                f"Realsense depth recv lag: {lag:.3f} seconds",
                throttle_duration_sec=1.0,
            )
        with self.latest_realsense_depth_image_lock:
            self.latest_realsense_depth_image = depth_msg

    def realsense_rgb_cb(
        self,
        rgb_ros_image: Union[CompressedImage, Image],
    ):
        if self.verbose:
            start_time = self.get_clock().now()
            lag = (
                start_time - Time.from_msg(rgb_ros_image.header.stamp)
            ).nanoseconds / 1.0e9
            self.get_logger().info(
                f"Realsense RGB recv lag: {lag:.3f} seconds",
                throttle_duration_sec=1.0,
            )

        image = ros_msg_to_cv2_image(rgb_ros_image, self.cv_bridge)
        if isinstance(rgb_ros_image, CompressedImage):
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        for image_config_name in self.realsense_params:
            # Overlay the pointcloud *before* cropping/masking/rotating the image,
            # for consistent (de)projection and transformations
            if self.realsense_depth_ar:
                with self.latest_realsense_depth_image_lock:
                    depth_msg = self.latest_realsense_depth_image
                image = self.overlay_realsense_depth_ar(depth_msg, image)
            img = self.configure_images(image, self.realsense_params[image_config_name])
            # if self.aruco_markers: img = self.aruco_markers_callback(marker_msg, img)
            self.realsense_images[image_config_name] = img

        self.realsense_rgb_image = self.realsense_images[
            self.realsense_camera_perspective
        ]
        self.publish_compressed_msg(
            self.realsense_rgb_image, self.publisher_realsense_cmp
        )

        if self.verbose:
            lag = (self.get_clock().now() - start_time).nanoseconds / 1.0e9
            self.get_logger().info(
                f"Realsense RGB processing time: {lag:.3f} seconds",
                throttle_duration_sec=1.0,
            )

    def gripper_camera_cb(self, ros_image):
        self.process_gripper_image(ros_image)

    def gripper_realsense_depth_cb(
        self,
        depth_msg: Union[CompressedImage, Image, PointCloud2],
    ):
        if self.verbose:
            start_time = self.get_clock().now()
            lag = (
                start_time - Time.from_msg(depth_msg.header.stamp)
            ).nanoseconds / 1.0e9
            self.get_logger().info(
                f"Gripper Depth recv lag: {lag:.3f} seconds",
                throttle_duration_sec=1.0,
            )

        # with self.latest_gripper_realsense_depth_image_lock:
        self.latest_gripper_realsense_depth_image = depth_msg

    def gripper_realsense_rgb_cb(
        self,
        ros_image: Union[CompressedImage, Image],
    ):
        if self.verbose:
            start_time = self.get_clock().now()
            lag = (
                start_time - Time.from_msg(ros_image.header.stamp)
            ).nanoseconds / 1.0e9
            self.get_logger().info(
                f"Gripper RGB recv lag: {lag:.3f} seconds",
                throttle_duration_sec=1.0,
            )

        self.process_gripper_image(ros_image)

        if self.verbose:
            lag = (self.get_clock().now() - start_time).nanoseconds / 1.0e9
            self.get_logger().info(
                f"Gripper RGB processing time: {lag:.3f} seconds",
                throttle_duration_sec=1.0,
            )

    def process_gripper_image(
        self,
        ros_image: Union[CompressedImage, Image],
    ):
        image = ros_msg_to_cv2_image(ros_image, self.cv_bridge)
        if isinstance(ros_image, CompressedImage):
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if self.gripper_depth_ar:
            # with self.latest_gripper_realsense_depth_image_lock:
            depth_msg = self.latest_gripper_realsense_depth_image
            if depth_msg is not None:
                image = self.overlay_gripper_depth_ar(image, depth_msg)

        if self.expanded_gripper:
            # Compute and publish the expanded gripper image
            gripper_camera_rgb_image = self.configure_images(
                image, self.expanded_gripper_params[self.gripper_camera_perspective]
            )
            self.gripper_camera_rgb_image = self.rotate_image_around_center(
                gripper_camera_rgb_image, -1 * self.roll_value
            )
            self.publish_compressed_msg(
                self.gripper_camera_rgb_image, self.publisher_gripper_cmp
            )
        else:
            # Compute and publish the standard gripper image
            gripper_camera_rgb_image = self.configure_images(
                image, self.gripper_params[self.gripper_camera_perspective]
            )
            self.gripper_camera_rgb_image = self.rotate_image_around_center(
                gripper_camera_rgb_image, -1 * self.roll_value
            )
            self.publish_compressed_msg(
                self.gripper_camera_rgb_image, self.publisher_gripper_cmp
            )

    def overlay_gripper_depth_ar(
        self, image: npt.NDArray, depth_msg: Union[CompressedImage, Image, PointCloud2]
    ) -> npt.NDArray:
        """
        Overlays points within the graspable region of the gripper in the depth image.

        Note that this method does not require extrinsics calibration between the camera and the gripper,
        because it utilizes the aruco markers on the gripper.
        """
        if self.gripper_P is None:
            self.get_logger().warn(
                "Gripper camera projection matrix is not available. Skipping point cloud processing."
            )
            return image

        # Load the depth image
        if isinstance(depth_msg, (CompressedImage, Image)):
            depth_image = ros_msg_to_cv2_image(depth_msg, self.cv_bridge)
            pc_in_camera = depth_img_to_pointcloud(
                depth_image,
                f_x=self.gripper_P[0, 0],
                f_y=self.gripper_P[1, 1],
                c_x=self.gripper_P[0, 2],
                c_y=self.gripper_P[1, 2],
            )
        else:
            pc_in_camera = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(depth_msg)

        # Create the Aruco Detector
        if self.aruco_detector is None:
            aruco_parameters = cv2.aruco.DetectorParameters()
            aruco_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.aruco_detector = cv2.aruco.ArucoDetector(
                aruco_dictionary, aruco_parameters
            )

        # Detect the gripper markers
        corners, ids, _ = self.aruco_detector.detectMarkers(image)
        if ids is None:
            self.get_logger().debug(
                "Did not detect any aruco markers on the gripper. Skipping point cloud processing.",
                throttle_duration_sec=1.0,
            )
            return image
        aruco_center_pos = {}
        for label, aruco_id in self.gripper_aruco_ids.items():
            if aruco_id in ids:
                # Update the crop corners
                idx = np.argmax(ids == aruco_id)
                aruco_corners = corners[idx][0]
                center = np.mean(aruco_corners, axis=0)
                aruco_center_pos[label] = deproject_pixel_to_point(
                    center[0], center[1], pc_in_camera, self.gripper_P
                )
        if (
            "finger_left" not in aruco_center_pos
            or "finger_right" not in aruco_center_pos
        ):
            self.get_logger().debug(
                "Did not detect both aruco markers on the gripper. Skipping point cloud processing.",
                throttle_duration_sec=1.0,
            )
            return image

        # Filter the points to those in the range. Note that (x, y, z) is in the
        # camera frame (e.g., +z out of camera, +x to the left of camera, +y up)
        left_x, left_y, left_z = aruco_center_pos["finger_left"]
        right_x, right_y, right_z = aruco_center_pos["finger_right"]
        pcl_cloud = pcl.PointCloud(np.array(pc_in_camera, dtype=np.float32))
        # Filter points within the distance range. Add a depth offset of 5cm to
        # account for the offset between the aruco marker and the gripper tip.
        z_offset_m = 0.04
        passthrough_z = pcl_cloud.make_passthrough_filter()
        passthrough_z.set_filter_field_name("z")
        passthrough_z.set_filter_limits(0.01, max(left_z, right_z) + z_offset_m)
        pcl_cloud_filtered = passthrough_z.filter()
        # Filter points within the x range
        passthrough_x = pcl_cloud_filtered.make_passthrough_filter()
        passthrough_x.set_filter_field_name("x")
        passthrough_x.set_filter_limits(left_x, right_x)
        pcl_cloud_filtered = passthrough_x.filter()
        # Filter points within the y range
        y_offset_m = 0.02
        passthrough_y = pcl_cloud_filtered.make_passthrough_filter()
        passthrough_y.set_filter_field_name("y")
        passthrough_y.set_filter_limits(
            min(left_y, right_y) - y_offset_m, max(left_y, right_y) + y_offset_m
        )
        pcl_cloud_filtered = passthrough_y.filter()

        # Convert filtered points to (u, v) indices
        pts_in_range = pcl_cloud_filtered.to_array()
        pts_in_range = np.hstack((pts_in_range, np.ones((pts_in_range.shape[0], 1))))
        coords = np.matmul(self.gripper_P, np.transpose(pts_in_range))  # 3 x N
        u_idx = (coords[0, :] / coords[2, :]).astype(int)
        v_idx = (coords[1, :] / coords[2, :]).astype(int)
        uv = np.vstack((u_idx, v_idx)).T  # N x 2
        uv_dedup = np.unique(uv, axis=0)
        in_bounds_idx = np.where(
            (uv_dedup[:, 0] >= 0)
            & (uv_dedup[:, 0] < image.shape[1])
            & (uv_dedup[:, 1] >= 0)
            & (uv_dedup[:, 1] < image.shape[0])
        )
        u_mask = uv_dedup[in_bounds_idx][:, 0]
        v_mask = uv_dedup[in_bounds_idx][:, 1]

        # Overlay the pixels in the robot's reach
        image = image.astype(np.float32)
        image[v_mask, u_mask, :] *= 1.0 - self.GRIPPER_DEPTH_AR_ALPHA
        image[v_mask, u_mask, :] += self.GRIPPER_DEPTH_AR_ALPHA * np.array(
            self.GRIPPER_DEPTH_AR_COLOR
        )
        image = image.astype(np.uint8)

        return image

    def navigation_camera_cb(self, ros_image):
        image = ros_msg_to_cv2_image(ros_image, self.cv_bridge)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.overhead_camera_rgb_image = self.configure_images(
            image, self.overhead_params[self.overhead_camera_perspective]
        )
        self.publish_compressed_msg(
            self.overhead_camera_rgb_image, self.publisher_overhead_cmp
        )

    def rotate_image_around_center(self, image, angle):
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, math.degrees(angle), 1.0)
        result = cv2.warpAffine(
            image,
            rot_mat,
            image.shape[1::-1],
            flags=cv2.INTER_LINEAR,
            borderValue=self.BACKGROUND_COLOR,
        )
        return result

    def joint_state_cb(self, joint_state):
        if "joint_wrist_roll" in joint_state.name:
            roll_index = joint_state.name.index("joint_wrist_roll")
            self.roll_value = joint_state.position[roll_index]

    def publish_compressed_msg(self, image, publisher):
        msg = cv2_image_to_ros_msg(image, compress=True, bridge=self.cv_bridge)
        msg.header.stamp = self.get_clock().now().to_msg()
        publisher.publish(msg)


if __name__ == "__main__":
    rclpy.init()
    print(sys.argv)
    node = ConfigureVideoStreams(
        params_file=sys.argv[1],
        has_beta_teleop_kit=sys.argv[2] == "True",
        use_overhead=sys.argv[3] == "True",
        use_realsense=sys.argv[4] == "True",
        use_gripper=sys.argv[5] == "True",
    )
    print("Publishing reconfigured video stream")
    # Use a MultiThreadedExecutor so that subscriptions, actions, etc. can be
    # processed in parallel.
    executor = MultiThreadedExecutor(num_threads=8)
    rclpy.spin(node, executor=executor)
