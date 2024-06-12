# Standard imports
import array
from typing import Optional, Union

# Third-party imports
import cv2
import numpy as np
import numpy.typing as npt
from cv_bridge import CvBridge
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
