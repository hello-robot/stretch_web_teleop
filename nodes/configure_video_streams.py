#!/usr/bin/env python3

# Standard Imports
import math
import threading
from typing import Dict, List, Optional, Tuple, Union

# Third-party Imports
import cv2
import message_filters
import numpy as np
import numpy.typing as npt
import pcl
import PyKDL  # TODO: PyKDL is only used for pointcloud transforms and can be replaced with numpy.
import rclpy
import ros2_numpy
import tf2_py as tf2
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from std_srvs.srv import SetBool
from stretch_controls import StretchControls


class ConfigureVideoStreams:
    """
    This class handles all the image streams for the web teleop node. This includes
    getting the latest images (and exposing them to other parts of the web teleop
    node), rotating/cropping/masking the images, and publishing the images for the
    web interface to consume.
    """

    BACKGROUND_COLOR = (200, 200, 200)

    def __init__(
        self,
        node: Node,
        params_file: str,
        has_beta_teleop_kit: bool,
        tf_buffer: tf2_ros.Buffer,
        stretch_controls: StretchControls,
    ):
        """
        Initialize the ConfigureVideoStreams class.

        Parameters
        ----------
        node: The ROS node that this class is a part of.
        params_file: The path to the YAML file containing the image parameters.
        has_beta_teleop_kit: Whether the robot has the beta teleop kit.
        tf_buffer: The tf2_ros.Buffer object to use for transforming point clouds.
        stretch_controls: The StretchControls object to use for getting the wrist roll value.
        """
        # Store the parameters
        self.node = node
        with open(params_file, "r") as params:
            self.image_params = yaml.safe_load(params)
        self.tf_buffer = tf_buffer
        self.stretch_controls = stretch_controls

        # Loaded params for each video stream
        self.overhead_params = (
            self.image_params["overhead"] if "overhead" in self.image_params else None
        )
        self.realsense_params = (
            self.image_params["realsense"] if "realsense" in self.image_params else None
        )
        self.gripper_params = (
            self.image_params["gripper"] if "gripper" in self.image_params else None
        )
        self.expanded_gripper_params = (
            self.image_params["expandedGripper"]
            if "expandedGripper" in self.image_params
            else None
        )

        # Store the raw ROS messages. This is because processing the images takes
        # longer than the rate at which they are published, so we need to store them
        # and process them in a separate thread.
        self.latest_realsense_msgs_lock = threading.Lock()
        self.latest_realsense_msgs: Optional[Tuple[Image, PointCloud2]] = None
        self.latest_overhead_msgs_lock = threading.Lock()
        self.latest_overhead_msgs: Optional[Image] = None
        self.latest_gripper_msgs_lock = threading.Lock()
        self.latest_gripper_msgs: Optional[Image] = None

        self.realsense_rgb_image = None
        self.overhead_camera_rgb_image = None
        self.gripper_camera_rgb_image = None
        self.expanded_gripper_camera_rgb_image = None
        self.cv_bridge = CvBridge()

        # Stores the camera projection matrix
        self.P_lock = threading.Lock()
        self.P = None

        # Compressed Image publishers
        self.publisher_realsense_cmp = self.node.create_publisher(
            CompressedImage, "/camera/color/image_raw/rotated/compressed", 15
        )
        self.publisher_overhead_cmp = self.node.create_publisher(
            CompressedImage, "/navigation_camera/image_raw/rotated/compressed", 15
        )
        self.publisher_gripper_cmp = self.node.create_publisher(
            CompressedImage, "/gripper_camera/image_raw/cropped/compressed", 15
        )
        self.publisher_expanded_gripper_cmp = self.node.create_publisher(
            CompressedImage, "/gripper_camera/image_raw/expanded/compressed", 15
        )

        # Subscribers
        # TODO: Move these to compressed image subscribers!
        self.camera_rgb_subscriber = message_filters.Subscriber(
            self.node,
            Image,
            "/camera/color/image_raw",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        # self.overhead_camera_rgb_subscriber = self.node.create_subscription(
        #     Image,
        #     "/navigation_camera/image_raw",
        #     self.navigation_camera_cb,
        #     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        #     callback_group=MutuallyExclusiveCallbackGroup(),
        # )
        # self.gripper_camera_rgb_subscriber = self.node.create_subscription(
        #     Image,
        #     "/gripper_camera/image_raw",
        #     self.gripper_camera_cb,
        #     1,
        #     callback_group=MutuallyExclusiveCallbackGroup(),
        # )
        self.point_cloud_subscriber = message_filters.Subscriber(
            self.node,
            PointCloud2,
            "/camera/depth/color/points",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.got_camera_info = False  # We only need the camera info once
        self.camera_info_subscriber = self.node.create_subscription(
            CameraInfo,
            "/camera/aligned_depth_to_color/camera_info",
            self.camera_info_cb,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.camera_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [
                self.camera_rgb_subscriber,
                self.point_cloud_subscriber,
            ],
            1,
            1,
            allow_headerless=True,
        )
        self.camera_synchronizer.registerCallback(self.realsense_cb)

        # Default image perspectives
        # self.node.get_logger().info(f"wide_angle_cam={wide_angle_cam} d405={d405}")
        self.overhead_camera_perspective = (
            "fixed" if has_beta_teleop_kit else "wide_angle_cam"
        )
        self.realsense_camera_perspective = "default"
        self.gripper_camera_perspective = "default" if has_beta_teleop_kit else "d405"
        self.node.get_logger().info(self.gripper_camera_perspective)

        # Service for enabling the depth AR overlay on the realsense stream
        self.depth_ar_service = self.node.create_service(
            SetBool, "depth_ar", self.depth_ar_callback
        )
        self.depth_ar = False
        self.pcl_cloud_filtered = None

        self.roll_value = 0.0

    def initialize(self) -> bool:
        """
        Initialize parts of this class that require ROS to be spinning in the background.
        """
        # These threads will automatically terminate when the node does, since the iteration is
        # guared by rclpy.ok()
        self.process_realsense_msgs_thread = threading.Thread(
            target=self.process_realsense_msgs, daemon=True
        )
        self.process_realsense_msgs_thread.start()
        # self.process_overhead_msgs_thread = threading.Thread(
        #     target=self.process_overhead_msgs, daemon=True
        # )
        # self.process_overhead_msgs_thread.start()
        # self.process_gripper_msgs_thread = threading.Thread(
        #     target=self.process_gripper_msgs, daemon=True
        # )
        # self.process_gripper_msgs_thread.start()

        return True

    # https://github.com/ros/geometry2/blob/noetic-devel/tf2_sensor_msgs/src/tf2_sensor_msgs/tf2_sensor_msgs.py#L44
    def transform_to_kdl(self, t: TransformStamped) -> PyKDL.Frame:
        """
        Convert a TransformStamped message to a PyKDL.Frame object.

        Parameters
        ----------
        t: The TransformStamped message to convert.

        Returns
        -------
        PyKDL.Frame: The corresponding PyKDL.Frame object.
        """
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
    def do_transform_cloud(
        self, cloud: pcl.PointCloud, transform: TransformStamped
    ) -> npt.NDArray[float]:
        """
        Transform a point cloud using a TransformStamped message.

        Parameters
        ----------
        cloud: The point cloud to transform.
        transform: The TransformStamped message to use for the transformation.

        Returns
        -------
        np.ndarray: The transformed point cloud.
        """
        t_kdl = self.transform_to_kdl(transform)
        points_out = []
        points = cloud.to_array()
        for p_in in points:
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0], p_out[1], p_out[2]])
        return np.array(points_out)

    def camera_info_cb(self, msg: CameraInfo) -> None:
        """
        Callback for the camera info subscriber. Stores the camera projection matrix.

        Parameters
        ----------
        msg: The CameraInfo message containing the camera projection matrix.
        """
        if not self.got_camera_info:
            with self.P_lock:
                self.P = np.array(msg.p).reshape(3, 4)
            self.got_camera_info = True

    def get_realsense_projection_matrix(self) -> Optional[np.ndarray]:
        """
        Get the realsense camera projection matrix.

        Returns
        -------
        Optional[np.ndarray]: The realsense camera projection matrix.
        """
        with self.P_lock:
            return self.P

    def pc_callback(
        self, msg: PointCloud2, img: npt.NDArray[np.uint8]
    ) -> npt.NDArray[np.uint8]:
        """
        Uses a heuristic to determine which points in the point cloud are in the robot's
        reach, and changes the color of the corresponding pixels in the image. The heuristic
        keeps points that are within 0.01m to 1.5m from the camera, and within 0.25m to 1m
        from the robot in the XY plane.

        Parameters
        ----------
        msg: The PointCloud2 message containing the point cloud.
        img: The image to change the color of.

        Returns
        -------
        np.ndarray: The image with the color of the pixels in the robot's reach changed.
        """
        with self.P_lock:
            P = self.P
        if P is None:
            self.node.get_logger().warn(
                "Camera projection matrix is not available. Skipping point cloud processing."
            )
            return img

        pc_in_camera = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
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
            self.node.get_logger().warn(
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
            coords = np.matmul(P, np.transpose(pts_in_range))  # 3 x N
            u_idx = (coords[0, :] / coords[2, :]).astype(int)
            v_idx = (coords[1, :] / coords[2, :]).astype(int)
            in_bounds_idx = np.where(
                (u_idx >= 0)
                & (u_idx < img.shape[1])
                & (v_idx >= 0)
                & (v_idx < img.shape[0])
            )
            # Color the pixels in the robot's reach
            img[v_idx[in_bounds_idx], u_idx[in_bounds_idx], :] = [0, 191, 255, 50]

        return img

    def depth_ar_callback(
        self, req: SetBool.Request, res: SetBool.Response
    ) -> SetBool.Response:
        """
        Callback for the depth augmented reality (AR) service. Enables or disables the
        depth AR overlay on the realsense RGB stream.

        Parameters
        ----------
        req: The request object containing the data.
        res: The response object to populate with the result.

        Returns
        -------
        SetBool.Response: The response object.
        """
        self.depth_ar = req.data
        res.success = True
        return res

    def crop_image(
        self, image: npt.NDArray[np.uint8], params: Dict
    ) -> npt.NDArray[np.uint8]:
        """
        Crop an image.

        Parameters
        ----------
        image: The image to crop.
        params: The parameters for cropping the image.

        Returns
        -------
        np.ndarray: The cropped image.
        """
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

    def inverse_crop_pixel(self, u: int, v: int, params: Dict) -> Tuple[int, int]:
        """
        Takes in pixel coordinate (u, v) in the **cropped** image and
        returns the pixel coordinate in the **original** image.

        Parameters
        ----------
        u: The x-coordinate of the pixel in the cropped image.
        v: The y-coordinate of the pixel in the cropped image.
        params: The parameters for cropping the image.

        Returns
        -------
        Tuple[int, int]: The pixel coordinate in the original image.
        """
        if params["x_min"] is None:
            raise ValueError("Crop x_min is not defined!")
        if params["y_min"] is None:
            raise ValueError("Crop y_min is not defined!")
        if "x_min" in params["crop"]:
            u += params["crop"]["x_min"]
        if "y_min" in params["crop"]:
            v += params["crop"]["y_min"]
        return u, v

    # https://stackoverflow.com/questions/44865023/how-can-i-create-a-circular-mask-for-a-numpy-array
    def create_circular_mask(
        self,
        h: int,
        w: int,
        center: Optional[Tuple[int, int]] = None,
        radius: Optional[int] = None,
    ) -> npt.NDArray[np.bool]:
        """
        Create a circular mask.

        Parameters
        ----------
        h: The height of the mask.
        w: The width of the mask.
        center: The center of the circle. If None, use half-way between the width and height.
        radius: The radius of the circle. If None, use the smallest length between the center
            and an edge.

        Returns
        -------
        np.ndarray: The circular image.
        """
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

    def mask_image(
        self, image: npt.NDArray[np.uint8], params: Dict
    ) -> npt.NDArray[np.uint8]:
        """
        Mask an image.

        Parameters
        ----------
        image: The image to mask.
        params: The parameters for masking the image.

        Returns
        -------
        np.ndarray: The masked image.
        """
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

    def rotate_image(
        self, image: npt.NDArray[np.uint8], value: str
    ) -> npt.NDArray[np.uint8]:
        """
        Rotate an image.

        Parameters
        ----------
        image: The image to rotate.
        value: The value to rotate the image by. Either ROTATE_90_CLOCKWISE, ROTATE_180, or
            ROTATE_90_COUNTERCLOCKWISE.

        Returns
        -------
        np.ndarray: The rotated image.

        Raises
        ------
        ValueError: If the value is not one of ROTATE_90_CLOCKWISE, ROTATE_180, or
            ROTATE_90_COUNTERCLOCKWISE.
        """
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

    def inverse_rotate_pixel(
        self, u: int, v: int, w: int, h: int, params: Dict
    ) -> Tuple[int, int]:
        """
        Takes in pixel coordinate (u, v) in the **rotated** image and
        returns the pixel coordinate in the **original** image.

        Parameters
        ----------
        u: The x-coordinate of the pixel in the rotated image.
        v: The y-coordinate of the pixel in the rotated image.
        w: The width of the original image.
        h: The height of the rotoriginalated image.
        params: The parameters for rotating the image.

        Returns
        -------
        Tuple[int, int]: The pixel coordinate in the original image.
        """
        if params["rotate"] == "ROTATE_90_CLOCKWISE":
            u, v = v, h - u - 1
        elif params["rotate"] == "ROTATE_180":
            u, v = w - u - 1, h - v - 1
        elif params["rotate"] == "ROTATE_90_COUNTERCLOCKWISE":
            u, v = w - v - 1, u
        else:
            raise ValueError(
                "Invalid rotate image value: options are ROTATE_90_CLOCKWISE, ROTATE_180, or ROTATE_90_COUNTERCLOCKWISE"
            )
        return u, v

    def unscale_pixel(
        self, scaled_u: float, scaled_v: float, w: int, h: int, params: Dict
    ) -> Tuple[int, int]:
        """
        Given a scaled pixel in a transformed image and the width and height of the
        original image, return the pixel in the original image.

        Parameters
        ----------
        scaled_u: The scaled x-coordinate of the pixel in the transformed image, in [0.0, 1.0]
        scaled_v: The scaled y-coordinate of the pixel in the transformed image, in [0.0, 1.0]
        w: The width of the original image.
        h: The height of the original image.
        params: The parameters for configuring the image.

        Returns
        -------
        Tuple[int, int]: The pixel coordinate in the original image.
        """
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
        return int(u), int(v)

    def configure_images(
        self, bgr_image: npt.NDArray[np.uint8], params: Dict
    ) -> npt.NDArray[np.uint8]:
        """
        Configure an image by cropping, masking, and/or rotating it, as specified by the
        parameters. Also converts the image to RGB(A).

        Parameters
        ----------
        bgr_image: The image to configure, in BGR.
        params: The parameters for configuring the image.

        Returns
        -------
        np.ndarray: The configured image, in RGB.
        """
        color_transform = (
            cv2.COLOR_BGR2RGB if bgr_image.shape[-1] == 3 else cv2.COLOR_BGRA2RGBA
        )
        rgb_image = cv2.cvtColor(bgr_image, color_transform)
        if params:
            if params["crop"]:
                rgb_image = self.crop_image(rgb_image, params["crop"])
            if params["mask"]:
                rgb_image = self.mask_image(rgb_image, params["mask"])
            if params["rotate"]:
                rgb_image = self.rotate_image(rgb_image, params["rotate"])
        return rgb_image

    def inverse_transform_pixel(
        self,
        u: Union[int, float],
        v: Union[int, float],
        param_keys: List[str],
        scaled: bool = False,
    ) -> Tuple[bool, int, int]:
        """
        Given a (scaled) pixel coordinate in a transformed image, return the pixel coordinate
        in the original image.

        Parameters
        ----------
        u: The (scaled) x-coordinate of the pixel in the transformed image.
        v: The (scaled) y-coordinate of the pixel in the transformed image.
        param_keys: The possibly-nested keys in the image parameters. E.g., if this is
            ["realsense", "default"], then the parameters self.image_params["realsense"]["default"]
            will be used. If at any point the key doesn't exist, the parameters are assumed to
            be empty. Note that the first element is expected to be either "realsense", "overhead",
            "gripper", or "expandedGripper".
        scaled: Whether the pixel coordinates are scaled. If True, the pixel coordinates are
            scaled to the range [0, 1].

        Returns
        -------
        Tuple[bool, int, int]: Whether it succeded, and the pixel coordinate in the original image.
        """
        # Get the camera name
        if len(param_keys) == 0:
            raise ValueError("param_keys must have at least one element")
        camera_name = param_keys[0]

        # Get the params
        param_key_i = 0
        params = self.image_params
        while param_key_i < len(param_keys):
            if param_keys[param_key_i] in params:
                params = params[param_keys[param_key_i]]
            else:
                params = None
                break
            param_key_i += 1

        # Get the width and height of the **original** image
        if camera_name == "realsense":
            latest_realsense_msgs = self.get_latest_realsense_msgs()
            if latest_realsense_msgs is None:
                return False, 0, 0
            w = latest_realsense_msgs[0].width
            h = latest_realsense_msgs[0].height
        else:
            raise NotImplementedError(
                "Getting the width and height of non-realsense cameras is not yet implemented"
            )

        # Unscale the pixel coordinates
        if scaled:
            (
                u,
                v,
            ) = self.unscale_pixel(float(u), float(v), w, h, params)
        else:
            u = int(u)
            v = int(v)
        self.node.get_logger().info(f"After scaling u={u} v={v}")

        # Undo the crop
        if params is not None and "crop" in params and params["crop"] is not None:
            u, v = self.inverse_crop_pixel(u, v, params)
        self.node.get_logger().info(f"After crop u={u} v={v}")

        # Note: We do not have to undo masking since it doesn't impact pixel coordinates.

        # Undo the rotation
        if params is not None and "rotate" in params and params["rotate"] is not None:
            u, v = self.inverse_rotate_pixel(u, v, w, h, params)
        self.node.get_logger().info(f"After rotate u={u} v={v}")

        return True, u, v

    def realsense_cb(self, ros_image: Image, pc: PointCloud2) -> None:
        """
        Callback for the realsense camera subscriber. Configures the images,
        overlays the pointcloud if that mode is enabled, and publishes the
        compressed image.

        Parameters
        ----------
        ros_image: The ROS Image message containing the realsense image.
        pc: The ROS PointCloud2 message containing the realsense point cloud.
        """
        with self.latest_realsense_msgs_lock:
            self.latest_realsense_msgs = (ros_image, pc)

    def process_realsense_msgs(self, rate_hz: float = 15.0) -> None:
        """
        Loop and process the realsense msgs. The difference between this and
        a standard callback is that here, even if it takes longer than rate_hz
        to process the msgs, it will not fill up the queue. Although in theory,
        the KEEP_LAST QoS is supposed to handle this, in practice I've experienced
        other messages (e.g., action invokations) being extremely delayed when the
        image queue is full.

        Parameters
        ----------
        rate_hz: The rate at which to process the messages.
        """
        rate = self.node.create_rate(rate_hz)
        while rclpy.ok():
            rate.sleep()

            with self.latest_realsense_msgs_lock:
                if self.latest_realsense_msgs is None:
                    continue
                ros_image, pc = self.latest_realsense_msgs
                self.latest_realsense_msgs = None

            image = self.cv_bridge.imgmsg_to_cv2(ros_image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
            for image_config_name in self.realsense_params:
                # Overlay the pointcloud *before* cropping/masking/rotating the image,
                # for consistent (de)projection and transformations
                if self.depth_ar:
                    image = self.pc_callback(pc, image)
                img = self.configure_images(
                    image, self.realsense_params[image_config_name]
                )
                # if self.aruco_markers: img = self.aruco_markers_callback(marker_msg, img)
                if image_config_name == self.realsense_camera_perspective:
                    self.realsense_rgb_image = img

            self.publish_compressed_msg(
                self.realsense_rgb_image, self.publisher_realsense_cmp
            )

    def get_latest_realsense_msgs(self) -> Optional[Tuple[Image, PointCloud2]]:
        """
        Get the latest realsense messages.

        Returns
        -------
        Optional[Tuple[Image, PointCloud2]]: The latest realsense messages.
        """
        with self.latest_realsense_msgs_lock:
            return self.latest_realsense_msgs

    def gripper_camera_cb(self, ros_image: Image):
        """
        Callback for the gripper camera subscriber. Configures the images and publishes
        the compressed image.

        Parameters
        ----------
        ros_image: The ROS Image message containing the gripper camera image.
        """
        with self.latest_gripper_msgs_lock:
            self.latest_gripper_msgs = ros_image

    def process_gripper_msgs(self, rate_hz: float = 15.0) -> None:
        """
        Loop and process the gripper msgs. The difference between this and
        a standard callback is that here, even if it takes longer than rate_hz
        to process the msgs, it will not fill up the queue. Although in theory,
        the KEEP_LAST QoS is supposed to handle this, in practice I've experienced
        other messages (e.g., action invokations) being extremely delayed when the
        image queue is full.

        Parameters
        ----------
        rate_hz: The rate at which to process the messages.
        """
        rate = self.node.create_rate(rate_hz)
        while rclpy.ok():
            rate.sleep()

            with self.latest_gripper_msgs_lock:
                if self.latest_gripper_msgs is None:
                    continue
                ros_image = self.latest_gripper_msgs
                self.latest_gripper_msgs = None

            # Get the wrist roll value
            roll_joint_name = "joint_wrist_roll"
            joint_states = self.stretch_controls.get_current_joints(
                joint_names={roll_joint_name}
            )
            if joint_states is None:
                roll_value = 0.0
            else:
                roll_value = joint_states[roll_joint_name]

            image = self.cv_bridge.imgmsg_to_cv2(ros_image, "rgb8")
            # Compute and publish the standard gripper image
            gripper_camera_rgb_image = self.configure_images(
                image, self.gripper_params[self.gripper_camera_perspective]
            )
            self.gripper_camera_rgb_image = self.rotate_image_around_center(
                gripper_camera_rgb_image, -1 * roll_value
            )
            self.publish_compressed_msg(
                self.gripper_camera_rgb_image, self.publisher_gripper_cmp
            )
            # Compute and publish the expanded gripper image
            expanded_gripper_camera_rgb_image = self.configure_images(
                image, self.expanded_gripper_params[self.gripper_camera_perspective]
            )
            self.expanded_gripper_camera_rgb_image = self.rotate_image_around_center(
                expanded_gripper_camera_rgb_image, -1 * roll_value
            )
            self.publish_compressed_msg(
                self.expanded_gripper_camera_rgb_image,
                self.publisher_expanded_gripper_cmp,
            )

    def navigation_camera_cb(self, ros_image: Image):
        """
        Callback for the overhead camera subscriber. Configures the images and publishes
        the compressed image.

        Parameters
        ----------
        ros_image: The ROS Image message containing the overhead camera image.
        """
        with self.latest_overhead_msgs_lock:
            self.latest_overhead_msgs = ros_image

    def process_overhead_msgs(self, rate_hz: float = 15.0) -> None:
        """
        Loop and process the overhead msgs. The difference between this and
        a standard callback is that here, even if it takes longer than rate_hz
        to process the msgs, it will not fill up the queue. Although in theory,
        the KEEP_LAST QoS is supposed to handle this, in practice I've experienced
        other messages (e.g., action invokations) being extremely delayed when the
        image queue is full.

        Parameters
        ----------
        rate_hz: The rate at which to process the messages.
        """

        rate = self.node.create_rate(rate_hz)

        while rclpy.ok():
            rate.sleep()

            with self.latest_overhead_msgs_lock:
                if self.latest_overhead_msgs is None:
                    continue
                ros_image = self.latest_overhead_msgs
                self.latest_overhead_msgs = None

            image = self.cv_bridge.imgmsg_to_cv2(ros_image, "rgb8")
            self.overhead_camera_rgb_image = self.configure_images(
                image, self.overhead_params[self.overhead_camera_perspective]
            )
            self.publish_compressed_msg(
                self.overhead_camera_rgb_image, self.publisher_overhead_cmp
            )

    def rotate_image_around_center(
        self, image: npt.NDArray[np.uint8], angle: float
    ) -> npt.NDArray[np.uint8]:
        """
        Rotate an image around its center.

        Parameters
        ----------
        image: The image to rotate.
        angle: The angle to rotate the image by (rad).

        Returns
        -------
        np.ndarray: The rotated image.
        """
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

    def publish_compressed_msg(
        self, image: npt.NDArray[np.uint8], publisher: Publisher
    ) -> None:
        """
        Publish a compressed image message.

        Parameters
        ----------
        image: The image to publish.
        publisher: The publisher to publish the image to.
        """
        msg = CompressedImage()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode(".jpg", image)[1]).tobytes()
        publisher.publish(msg)
