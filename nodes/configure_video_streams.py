#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
import message_filters
import numpy as np
import cv2
import yaml
import sys
import ros2_numpy 
import tf2_ros
import pcl
import PyKDL
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import Image, CameraInfo, CompressedImage, PointCloud2, CameraInfo
from sensor_msgs_py.point_cloud2 import read_points, create_cloud
from cv_bridge import CvBridge
from stretch_teleop_interface_msgs.srv import CameraPerspective, DepthAR, ArucoMarkers
from visualization_msgs.msg import MarkerArray
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ConfigureVideoStreams(Node):
    def __init__(self, params_file):
        super().__init__('configure_video_streams')
        
        with open(params_file, 'r') as params:
            self.image_params = yaml.safe_load(params)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=12))
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Loaded params for each video stream
        self.overhead_params = self.image_params["overhead"] if "overhead" in self.image_params else None
        self.realsense_params = self.image_params["realsense"] if "realsense" in self.image_params else None
        self.gripper_params = self.image_params["gripper"] if "gripper" in self.image_params else None

        # Stores all the images created using the loaded params
        self.overhead_images = {}
        self.realsense_images = {}
        self.gripper_images = {}

        self.realsense_rgb_image = None
        self.overhead_camera_rgb_image = None
        self.gripper_camera_rgb_image = None
        self.cv_bridge = CvBridge()

        # Compressed Image publishers
        self.publisher_realsense_cmp = self.create_publisher(CompressedImage, '/camera/color/image_raw/rotated/compressed', 15)
        self.publisher_overhead_cmp = self.create_publisher(CompressedImage, '/navigation_camera/image_raw/rotated/compressed', 15)
        self.publisher_gripper_cmp = self.create_publisher(CompressedImage, '/gripper_camera/image_raw/cropped/compressed', 15)

        # Subscribers
        self.camera_rgb_subscriber =  message_filters.Subscriber(self, Image, "/camera/color/image_raw")
        self.overhead_camera_rgb_subscriber = self.create_subscription(Image, "/navigation_camera/image_raw", self.navigation_camera_cb, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.gripper_camera_rgb_subscriber = self.create_subscription(Image, "/gripper_camera/image_raw", self.gripper_camera_cb, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.point_cloud_subscriber =  message_filters.Subscriber(self, PointCloud2, "/camera/depth/color/points")
        self.camera_info_subscriber = self.create_subscription(CameraInfo, "/camera/aligned_depth_to_color/camera_info", self.camera_info_cb, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.aruco_markers_subscriber = message_filters.Subscriber(self, MarkerArray, "/aruco/marker_array")
        self.camera_synchronizer = message_filters.ApproximateTimeSynchronizer([
            self.camera_rgb_subscriber, 
            self.point_cloud_subscriber,
        #     # self.aruco_markers_subscriber
        ], 1, 1, allow_headerless=True)
        self.camera_synchronizer.registerCallback(self.realsense_cb)

        # Service for requested image perspectives configured based on params file
        self.camera_perspective_service = self.create_service(CameraPerspective, 'camera_perspective', self.camera_perspective_callback)

        # Default image perspectives
        self.camera_perspective = {"overhead": "nav", "realsense": "default", "gripper": "default"}
        self.overhead_camera_perspective = 'nav'
        self.realsense_camera_perspective = 'default'
        self.gripper_camera_perspective = 'default'

        # Service for enabling the depth AR overlay on the realsense stream
        self.depth_ar_service = self.create_service(DepthAR, 'depth_ar', self.depth_ar_callback)
        self.depth_ar = False
        self.pcl_cloud_filtered = None

        # Service for enabling aruco marker detection
        self.aruco_markers_service = self.create_service(ArucoMarkers, 'aruco_markers', self.display_aruco_markers_callback)
        self.aruco_markers = False

    # https://github.com/ros/geometry2/blob/noetic-devel/tf2_sensor_msgs/src/tf2_sensor_msgs/tf2_sensor_msgs.py#L44
    def transform_to_kdl(self, t):
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                    t.transform.rotation.z, t.transform.rotation.w),
                        PyKDL.Vector(t.transform.translation.x, 
                                        t.transform.translation.y, 
                                        t.transform.translation.z))

    # https://github.com/ros/geometry2/blob/noetic-devel/tf2_sensor_msgs/src/tf2_sensor_msgs/tf2_sensor_msgs.py#L52
    def do_transform_cloud(self, cloud, transform):
        t_kdl = self.transform_to_kdl(transform)
        points_out = []
        points = cloud.to_array()
        for p_in in points:
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0], p_out[1], p_out[2]])
        return np.array(points_out)
    
    def camera_info_cb(self, msg):
        self.P =  np.array(msg.p).reshape(3,4)
        # self.camera_info_subscriber.destroy()

    def pc_callback(self, msg, img):
        pc_in_camera = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        pcl_cloud = pcl.PointCloud(np.array(pc_in_camera, dtype=np.float32))
        passthrough = pcl_cloud.make_passthrough_filter()
        passthrough.set_filter_field_name("z")
        passthrough.set_filter_limits(0.01, 1.5)
        self.pcl_cloud_filtered = passthrough.filter()

        # Get transform
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                'camera_color_optical_frame', 
                Time(),
                timeout=Duration(seconds=0.1)
            )
        except:
            self.get_logger().warn("Could not find the transform between frames {} and {}".format('base_link', 'camera_color_optical_frame'))
            return img
        
        if not self.pcl_cloud_filtered: return img
        if self.pcl_cloud_filtered.to_array().size == 0: return img

        # Transform points cloud to base link and points that are in robot's reach
        pc_in_base_link = self.do_transform_cloud(self.pcl_cloud_filtered, transform)
        # pc_in_base_link = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_in_base_link_msg)
        dist = np.sqrt(np.power(pc_in_base_link[:,0], 2) + np.power(pc_in_base_link[:,1], 2))
        filtered_indices = np.where((dist > 0.25) & (dist < 1))[0]

        # Get filtered points in camera frame
        # pc_in_camera = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(msg) # N x 3
        pts_in_range = self.pcl_cloud_filtered.to_array()[filtered_indices, :]
        pts_in_range = np.hstack((pts_in_range, np.ones((pts_in_range.shape[0],1))))

        # Get pixel coordinates 
        coords = np.matmul(self.P, np.transpose(pts_in_range)) # 3 x N
        x_idx = np.absolute((coords[0,:]/coords[2,:]).astype(int))
        y_idx = np.absolute((coords[1,:]/coords[2,:]).astype(int))
        # negative_indices = np.where((x_idx < 0) or (y_idx < 0))

        # Change color of pixels in robot's reach
        if img is not None:
            img[x_idx, img.shape[1] - 1 - y_idx, :] = [255, 191, 0, 50]
            # img[y_idx, x_idx, 3] = 0

        return img 
    
    def aruco_markers_callback(self, msg, img):
        markers = msg.markers
        for marker in markers:
            position = marker.pose.position

            center = np.matmul(self.P, np.transpose(np.array([position.x, position.y, position.z, 1]))) # 3 x 1
            cx = (center[0]/center[2]).astype(int)
            cy = (center[1]/center[2]).astype(int)
            r = 25

            # img[circle_coords] = [255, 191, 0]
            mask = self.create_circular_mask(img.shape[0], img.shape[1], (cy, cx), 20)
            masked_img = img.copy()
            masked_img[np.fliplr(mask)] = [255, 191, 0, 50]
            img = cv2.addWeighted(masked_img, 0.4, img, 0.5, 0)

        return img

    def camera_perspective_callback(self, req, res):
        if req.camera not in self.camera_perspective: 
            raise ValueError("CameraPerspective.srv camera must be overhead, realsense or gripper") 
        
        try: 
            self.camera_perspective[req.camera] = req.perspective
            res.success = True
            return res
        except:
            res.success = False
            return res

    def depth_ar_callback(self, req, res):
        print('depth')
        self.depth_ar = req.enable
        res.success = True
        return res

    def display_aruco_markers_callback(self, req, res):
        self.aruco_markers = req.enable
        res.success = True
        return res

    def crop_image(self, image, params):
        if params["x_min"] is None: raise ValueError("Crop x_min is not defined!")
        if params["x_max"] is None: raise ValueError("Crop x_max is not defined!") 
        if params["y_min"] is None: raise ValueError("Crop y_min is not defined!") 
        if params["y_max"] is None: raise ValueError("Crop y_max is not defined!") 

        x_min = params["x_min"]
        x_max = params["x_max"]
        y_min = params["y_min"]
        y_max = params["y_max"]

        return image[x_min:x_max, y_min:y_max]

    # https://stackoverflow.com/questions/44865023/how-can-i-create-a-circular-mask-for-a-numpy-array
    def create_circular_mask(self, h, w, center=None, radius=None):
        if center is None: # use the middle of the image
            center = (int(w/2), int(h/2))
        if radius is None: # use the smallest distance between the center and image walls
            radius = min(center[0], center[1], w-center[0], h-center[1])

        Y, X = np.ogrid[:h, :w]
        dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)

        mask = dist_from_center <= radius
        return mask

    def mask_image(self, image, params):
        if params["width"] is None: raise ValueError("Mask width is not defined!") 
        if params["height"] is None: raise ValueError("Mask height is not defined!")

        w = params["width"]
        h = params["height"]
        center = (params["center"]["x"], params["center"]["y"]) if params["center"] else None
        radius = params["radius"]

        mask = self.create_circular_mask(h, w, center, radius)
        img = image.copy()
        img[~mask] = 200
        return img

    def rotate_image(self, image, value):
        if value == 'ROTATE_90_CLOCKWISE':
            return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif value == "ROTATE_180":
            return cv2.rotate(image, cv2.ROTATE_180)
        elif value == "ROTATE_90_COUNTERCLOCKWISE":
            return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            raise ValueError("Invalid rotate image value: options are ROTATE_90_CLOCKWISE, ROTATE_180, or ROTATE_90_COUNTERCLOCKWISE")

    def configure_images(self, ros_image, params):
        rgb_image = self.cv_bridge.imgmsg_to_cv2(ros_image)
        if rgb_image.shape[-1] == 2:
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_YUV2RGB_YUYV)
        else:
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        if params:
            if params['crop']:
                rgb_image = self.crop_image(rgb_image, params['crop'])
            if params['mask']:
                rgb_image = self.mask_image(rgb_image, params['mask'])
            if params['rotate']:
                rgb_image = self.rotate_image(rgb_image, params['rotate'])
        return rgb_image

    def realsense_cb(self, image, pc):
        for image_config_name in self.realsense_params:
            img = self.configure_images(image, self.realsense_params[image_config_name])
            img = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA)
            if self.depth_ar: img = self.pc_callback(pc, img)
            # if self.aruco_markers: img = self.aruco_markers_callback(marker_msg, img)
            self.realsense_images[image_config_name] = img
        
        self.realsense_rgb_image = self.realsense_images[self.camera_perspective["realsense"]]
        self.publish_compressed_msg(self.realsense_rgb_image, self.publisher_realsense_cmp)

    def gripper_camera_cb(self, image):
        for image_config_name in self.gripper_params:
            self.gripper_images[image_config_name] = \
                self.configure_images(image, self.gripper_params[image_config_name])
        
        self.gripper_camera_rgb_image = self.gripper_images[self.camera_perspective["gripper"]]
        self.publish_compressed_msg(self.gripper_camera_rgb_image, self.publisher_gripper_cmp)

    def navigation_camera_cb(self, image):
        for image_config_name in self.overhead_params:
            self.overhead_images[image_config_name] = \
                self.configure_images(image, self.overhead_params[image_config_name])

        self.overhead_camera_rgb_image = self.overhead_images[self.camera_perspective["overhead"]]
        self.publish_compressed_msg(self.overhead_camera_rgb_image, self.publisher_overhead_cmp)

    def publish_compressed_msg(self, image, publisher):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
        publisher.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    node = ConfigureVideoStreams(sys.argv[1])
    print("Publishing reconfigured video stream")
    rclpy.spin(node)