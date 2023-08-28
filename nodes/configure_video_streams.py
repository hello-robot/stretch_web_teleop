#!/usr/bin/env python3

import rospy
import message_filters
import numpy as np
import cv2
import yaml
import sys
import ros_numpy 
import pcl 
import tf2_ros 
import compressed_image_transport 

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import Image, CameraInfo, CompressedImage, PointCloud2, CameraInfo
from cv_bridge import CvBridge
from stretch_teleop_interface.srv import CameraPerspective, DepthAR, ArucoMarkers
from visualization_msgs.msg import MarkerArray

class ConfigureVideoStreams:
    def __init__(self, params_file):
        with open(params_file, 'r') as params:
            self.image_params = yaml.safe_load(params)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
        tf2_ros.TransformListener(self.tf_buffer)
        
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
        self.publisher_realsense_cmp = \
            rospy.Publisher('/camera/color/image_raw/rotated/compressed', CompressedImage, queue_size=10)
        self.publisher_overhead_cmp = \
            rospy.Publisher('/navigation_camera/image_raw/rotated/compressed', CompressedImage, queue_size=10)
        self.publisher_gripper_cmp = \
            rospy.Publisher('/gripper_camera/image_raw/cropped/compressed', CompressedImage, queue_size=10)

        # Subscribers
        self.camera_rgb_subscriber = message_filters.Subscriber(f'/camera/color/image_raw', Image)
        self.overhead_camera_rgb_subscriber = message_filters.Subscriber(f'/navigation_camera/image_raw', Image)
        self.gripper_camera_rgb_subscriber = message_filters.Subscriber(f'/gripper_camera/image_raw', Image)
        self.point_cloud_subscriber = message_filters.Subscriber("/voxel_grid/output", PointCloud2)
        self.camera_info_subscriber = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo)
        self.aruco_markers_subscriber = message_filters.Subscriber("/aruco/marker_array", MarkerArray)
        self.camera_synchronizer = message_filters.ApproximateTimeSynchronizer([
            self.camera_rgb_subscriber, 
            self.overhead_camera_rgb_subscriber, 
            self.gripper_camera_rgb_subscriber, 
            self.point_cloud_subscriber,
            self.camera_info_subscriber,
            self.aruco_markers_subscriber
        ], 1, 1, allow_headerless=True)
        self.camera_synchronizer.registerCallback(self.camera_callback)

        # Service for requested image perspectives configured based on params file
        self.camera_perspective_service = rospy.Service('camera_perspective', CameraPerspective, self.camera_perspective_callback)

        # Default image perspectives
        self.camera_perspective = {"overhead": "nav", "realsense": "default", "gripper": "default"}
        self.overhead_camera_perspective = 'nav'
        self.realsense_camera_perspective = 'default'
        self.gripper_camera_perspective = 'default'

        # Service for enabling the depth AR overlay on the realsense stream
        self.depth_ar_service = rospy.Service('depth_ar', DepthAR, self.depth_ar_callback)
        self.depth_ar = False

        # Service for enable aruco marker detection
        self.aruco_markers_service = rospy.Service('aruco_markers', ArucoMarkers, self.display_aruco_markers_callback)
        self.aruco_markers = False

    def pc_callback(self, msg, img):
        # Get transform
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                'camera_color_optical_frame', 
                rospy.Time(0)
            )
        except tf2_ros.TransformException as e:
            print(e)
            return img
        
        # Transform points cloud to base link and points that are in robot's reach
        pc_in_base_link_msg = do_transform_cloud(msg, transform)
        pc_in_base_link = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_in_base_link_msg)
        dist = np.sqrt(np.power(pc_in_base_link[:,0], 2) + np.power(pc_in_base_link[:,1], 2))
        filtered_indices = np.where((dist > 0.25) & (dist < 1))[0]

        # Get filtered points in camera frame
        pc_in_camera = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg) # N x 3
        pts_in_range = pc_in_camera[filtered_indices, :]
        pts_in_range = np.hstack((pts_in_range, np.ones((pts_in_range.shape[0],1))))

        # Get pixel coordinates 
        coords = np.matmul(self.P, np.transpose(pts_in_range)) # 3 x N
        x_idx = (coords[0,:]/coords[2,:]).astype(int)
        y_idx = (coords[1,:]/coords[2,:]).astype(int)

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

    def camera_perspective_callback(self, req):
        if req.camera not in self.camera_perspective: 
            raise ValueError("CameraPerspective.srv camera must be overhead, realsense or gripper") 
        
        try: 
            self.camera_perspective[req.camera] = req.perspective
            return True
        except:
            return False

    def depth_ar_callback(self, req):
        self.depth_ar = req.enable
        return True

    def display_aruco_markers_callback(self, req):
        self.aruco_markers = req.enable
        return True

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
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        if params:
            if params['crop']:
                rgb_image = self.crop_image(rgb_image, params['crop'])
            if params['mask']:
                rgb_image = self.mask_image(rgb_image, params['mask'])
            if params['rotate']:
                rgb_image = self.rotate_image(rgb_image, params['rotate'])
        return rgb_image

    def camera_callback(self, ros_rgb_image, ros_overhead_rgb_image, ros_gripper_rgb_image, pc_msg, camera_info, marker_msg):
        self.P =  np.array(camera_info.P).reshape(3,4)
        for image_config_name in self.realsense_params:
            img = self.configure_images(ros_rgb_image, self.realsense_params[image_config_name])
            img = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA)
            if self.depth_ar: img = self.pc_callback(pc_msg, img)
            if self.aruco_markers: img = self.aruco_markers_callback(marker_msg, img)
            self.realsense_images[image_config_name] = img
        for image_config_name in self.overhead_params:
            self.overhead_images[image_config_name] = \
                self.configure_images(ros_overhead_rgb_image, self.overhead_params[image_config_name])
        for image_config_name in self.gripper_params:
            self.gripper_images[image_config_name] = \
                self.configure_images(ros_gripper_rgb_image, self.gripper_params[image_config_name])
        
        self.overhead_camera_rgb_image = self.overhead_images[self.camera_perspective["overhead"]]
        self.realsense_rgb_image = self.realsense_images[self.camera_perspective["realsense"]]
        self.gripper_camera_rgb_image = self.gripper_images[self.camera_perspective["gripper"]]

    def publish_compressed_msg(self, image, publisher):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
        publisher.publish(msg)

    def start(self):
        print("Publishing reconfigured video stream")
        while not rospy.is_shutdown():
            if self.overhead_camera_rgb_image is not None: 
                self.publish_compressed_msg(self.overhead_camera_rgb_image, self.publisher_overhead_cmp)
            if self.realsense_rgb_image is not None: 
                self.publish_compressed_msg(self.realsense_rgb_image, self.publisher_realsense_cmp)
            if self.gripper_camera_rgb_image is not None: 
                self.publish_compressed_msg(self.gripper_camera_rgb_image, self.publisher_gripper_cmp)

            rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('configure_video_streams')
    node = ConfigureVideoStreams(sys.argv[1])
    node.start()