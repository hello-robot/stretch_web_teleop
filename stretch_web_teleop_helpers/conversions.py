# Standard imports
import array
from typing import Optional, Tuple, Union

# Third-party imports
import cv2
import numpy as np
import numpy.typing as npt
import tf2_py as tf2
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image

# The fixed header that ROS2 Humble's compressed depth image transport plugin prepends to
# the data. The exact value was empirically determined, but the below link shows the code
# that prepends additional data:
#
# https://github.com/ros-perception/image_transport_plugins/blob/5ef79d74c4347e6a2d151df63230d5fea1357137/compressed_depth_image_transport/src/codec.cpp#L337 # noqa: E501
__COMPRESSED_DEPTH_16UC1_HEADER = array.array(
    "B", [0, 0, 0, 0, 46, 32, 133, 4, 192, 24, 60, 78]
)


def ros_msg_to_cv2_image(
    msg: Union[Image, CompressedImage],
    bridge: Optional[CvBridge] = None,
) -> npt.NDArray:
    """
    Convert a ROS Image or CompressedImage message to a cv2 image. By default,
    this will maintain the depth of the image (e.g., 16-bit depth for depth
    images) and maintain the format. Any conversions should be done outside
    of this function.

    Adapted from https://github.com/personalrobotics/ada_feeding/blob/ros2-devel/ada_feeding_perception/ada_feeding_perception/helpers.py # noqa: E501

    Parameters
    ----------
    msg: the ROS Image or CompressedImage message to convert
    bridge: the CvBridge to use for the conversion. This is only used if `msg`
        is a ROS Image message. If `bridge` is None, a new CvBridge will be
        created.
    """
    if bridge is None:
        bridge = CvBridge()
    if isinstance(msg, Image):
        return bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    if isinstance(msg, CompressedImage):
        if "compressedDepth" in msg.format:  # compressed depth image
            encoding = msg.format[: msg.format.find(";")]
            if encoding.lower() != "16uc1":
                raise NotImplementedError(
                    f"Encoding ({encoding}) not yet supported for compressed depth images"
                )
            return cv2.imdecode(
                np.frombuffer(
                    msg.data[len(__COMPRESSED_DEPTH_16UC1_HEADER) :], np.uint8
                ),
                cv2.IMREAD_UNCHANGED,
            )
        return bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
    raise ValueError("msg must be a ROS Image or CompressedImage")


def cv2_image_to_ros_msg(
    image: npt.NDArray,
    compress: bool,
    bridge: Optional[CvBridge] = None,
    encoding: str = "passthrough",
) -> Union[Image, CompressedImage]:
    """
    Convert a cv2 image to a ROS Image or CompressedImage message. Note that this
    does not set the header of the message; that must be done outside of this
    function.

    NOTE: This has been tested with converting an 8-bit greyscale image to a
    CompressedImage message. It has not been tested in any other circumstance.

    Adapted from https://github.com/personalrobotics/ada_feeding/blob/ros2-devel/ada_feeding_perception/ada_feeding_perception/helpers.py # noqa: E501

    Parameters
    ----------
    image: the cv2 image to convert
    compress: whether or not to compress the image. If True, a CompressedImage
        message will be returned. If False, an Image message will be returned.
    bridge: the CvBridge to use for the conversion. This is only used if `msg`
        is a ROS Image message. If `bridge` is None, a new CvBridge will be
        created.
    encoding: the encoding to use for the ROS Image message. This is only used
        if `compress` is False.
    """
    if bridge is None:
        bridge = CvBridge()
    if compress:
        if "compressedDepth" in encoding:  # compressed depth image
            encoding = encoding[: encoding.find(";")]
            if encoding.lower() != "16uc1":
                raise NotImplementedError(
                    f"Encoding ({encoding}) not yet supported for compressed depth images"
                )
            success, data = cv2.imencode(
                ".png",
                image,
                # PNG compression 1 is the best speed setting.
                [cv2.IMWRITE_PNG_COMPRESSION, 1],
            )
            if not success:
                raise RuntimeError("Failed to compress image")
            msg = CompressedImage(
                format=encoding,
                data=__COMPRESSED_DEPTH_16UC1_HEADER.tobytes() + data.tobytes(),
            )
            return msg
        # Compressed RGB image
        return bridge.cv2_to_compressed_imgmsg(image, dst_format="jpeg")
    # If we get here, we're not compressing the image
    return bridge.cv2_to_imgmsg(image, encoding=encoding)


def depth_img_to_pointcloud(
    depth_image: npt.NDArray,
    f_x: float,
    f_y: float,
    c_x: float,
    c_y: float,
    unit_conversion: float = 1000.0,
    transform: Optional[npt.NDArray] = None,
    u_offset: int = 0,
    v_offset: int = 0,
) -> npt.NDArray:
    """
    Converts a depth image to a point cloud.

    Adapted from https://github.com/personalrobotics/ada_feeding/blob/ros2-devel/ada_feeding_perception/ada_feeding_perception/helpers.py # noqa: E501

    Parameters
    ----------
    depth_image: The depth image to convert to a point cloud.
    f_x: The focal length of the camera in the x direction, using the pinhole
        camera model.
    f_y: The focal length of the camera in the y direction, using the pinhole
        camera model.
    c_x: The x-coordinate of the principal point of the camera, using the pinhole
        camera model.
    c_y: The y-coordinate of the principal point of the camera, using the pinhole
        camera model.
    unit_conversion: The depth values are divided by this constant. Defaults to 1000,
        as RealSense returns depth in mm, but we want the pointcloud in m.
    transform: An optional transform to apply to the point cloud. If set, this should
        be a 4x4 matrix.
    u_offset: An offset to add to the column index of every pixel in the depth
        image. This is useful if the depth image was cropped.
    v_offset: An offset to add to the row index of every pixel in the depth
        image. This is useful if the depth image was cropped.

    Returns
    -------
    pointcloud: The point cloud representation of the depth image.
    """
    # Although we could reduce it by passing in a camera matrix, I prefer to
    # keep the arguments explicit.

    # Get the pixel coordinates
    pixel_coords = np.mgrid[: depth_image.shape[0], : depth_image.shape[1]]
    pixel_coords[0] += v_offset
    pixel_coords[1] += u_offset

    # Mask out values outside the depth range
    mask = depth_image > 0
    depth_values = depth_image[mask]
    pixel_coords = pixel_coords[:, mask]

    # Convert units (e.g., mm to m)
    depth_values = np.divide(depth_values, unit_conversion)

    # Convert to 3D coordinates
    pointcloud = np.zeros((depth_values.shape[0], 3))
    pointcloud[:, 0] = np.multiply(pixel_coords[1] - c_x, np.divide(depth_values, f_x))
    pointcloud[:, 1] = np.multiply(pixel_coords[0] - c_y, np.divide(depth_values, f_y))
    pointcloud[:, 2] = depth_values

    # Apply the transform if it exists
    if transform is not None:
        pointcloud = np.hstack((pointcloud, np.ones((pointcloud.shape[0], 1))))
        pointcloud = np.dot(transform, pointcloud.T).T[:, :3]

    return pointcloud


def deproject_pixel_to_point(
    u: int, v: int, pointcloud: npt.NDArray[np.float32], proj: npt.NDArray[np.float32]
) -> Tuple[float, float, float]:
    """
    Deproject the clicked pixel to get the 3D coordinates of the clicked point.

    Parameters
    ----------
    u: The horizontal coordinate of the clicked pixel.
    v: The vertical coordinate of the clicked pixel.
    pointcloud: The pointcloud array of size (N, 3).
    proj: The camera's projection matrix of size (3, 4).

    Returns
    -------
    Tuple[float, float, float]: The 3D coordinates of the clicked point.
    """
    # Get the ray from the camera origin to the clicked point
    ray_dir = np.linalg.pinv(proj)[:3, :] @ np.array([u, v, 1])
    ray_dir /= np.linalg.norm(ray_dir)

    # Find the point that is closest to the ray
    p, r = pointcloud, ray_dir
    closest_point_idx = np.argmin(
        np.linalg.norm(
            p - np.multiply((p @ r).reshape((-1, 1)), r.reshape((1, 3))), axis=1
        )
    )

    return p[closest_point_idx]


def create_ros_pose(
    pos: npt.NDArray, quat: npt.NDArray, frame: Optional[str] = None
) -> Union[Pose, PoseStamped]:
    """
    Create a ROS Pose or PoseStamped message from a position and quaternion.

    Parameters
    ----------
    pos: The position of the pose
    quat: The quaternion of the pose, in XYZW.
    frame: The frame of the pose. If None, a Pose will eb returned, else a PoseStamped

    Returns
    -------
    pose: The ROS Pose or PoseStamped message
    """
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    if frame is None:
        return pose

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame
    pose_stamped.pose = pose
    return pose_stamped


def get_pos_quat_from_ros(
    pose: Union[Pose, PoseStamped]
) -> tuple[npt.NDArray, npt.NDArray]:
    """
    Get the position and quaternion from a ROS Pose or PoseStamped message.

    Parameters
    ----------
    pose: The ROS Pose or PoseStamped message

    Returns
    -------
    pos: The position of the pose
    quat: The quaternion of the pose, in XYZW.
    """
    if isinstance(pose, PoseStamped):
        pose = pose.pose

    pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    quat = np.array(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    )
    return pos, quat


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


def tf2_transform(
    tf_buffer: tf2_ros.Buffer,
    pose: PoseStamped,
    target_frame: str,
    timeout: Duration,
    verbose: bool = False,
) -> Tuple[bool, PoseStamped]:
    """
    Transform a pose to a target frame.

    Parameters
    ----------
    tf_buffer: The tf2_ros.Buffer object.
    pose: The pose to transform.
    target_frame: The target frame.
    timeout: The timeout.

    Returns
    -------
    Tuple[bool, PoseStamped]: Whether the transform was successful and the transformed pose.
    """
    try:
        pose_transformed = tf_buffer.transform(pose, target_frame, timeout=timeout)
    except (
        tf2.ConnectivityException,
        tf2.ExtrapolationException,
        tf2.InvalidArgumentException,
        tf2.LookupException,
        tf2.TimeoutException,
        tf2.TransformException,
    ):
        return False, PoseStamped()
    return True, pose_transformed


def tf2_get_transform(
    tf_buffer: tf2_ros.Buffer,
    target_frame: str,
    source_frame: str,
    timeout: Duration,
) -> Tuple[bool, TransformStamped]:
    """
    Get the transform from a source frame to a target frame.

    Parameters
    ----------
    tf_buffer: The tf2_ros.Buffer object.
    target_frame: The target frame.
    source_frame: The source frame.
    timeout: The timeout.

    Returns
    -------
    Tuple[bool, TransformStamped]: Whether the transform was successful and the transform message.
    """
    try:
        transform = tf_buffer.lookup_transform(
            target_frame, source_frame, Time(), timeout=timeout
        )
    except (
        tf2.ConnectivityException,
        tf2.ExtrapolationException,
        tf2.InvalidArgumentException,
        tf2.LookupException,
        tf2.TimeoutException,
        tf2.TransformException,
    ):
        return False, TransformStamped()
    return True, transform
