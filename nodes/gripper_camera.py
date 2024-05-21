#! /usr/bin/env python3


# my_image_publisher/my_image_publisher/publisher_node.py

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

UVC_COLOR_SIZE = [
    1024,
    768,
]  # [3840,2880] [1920, 1080] [1280, 720] [1280, 800] [640, 480]
UVC_FPS = 100

UVC_VIDEO_INDEX = "/dev/hello-gripper-camera"
UVC_VIDEO_FORMAT = "MJPG"  # MJPG YUYV

# brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
UVC_BRIGHTNESS = -40
# contrast 0x00980901 (int)    : min=0 max=64 step=1 default=32 value=32
UVC_CONTRAST = 40
# saturation 0x00980902 (int)    : min=0 max=128 step=1 default=90 value=90
UVC_SATURATION = 60
# hue 0x00980903 (int)    : min=-40 max=40 step=1 default=0 value=0
UVC_HUE = 0
# gamma 0x00980910 (int)    : min=72 max=500 step=1 default=100 value=100
UVC_GAMMA = 80
# gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=0
UVC_GAIN = 80
# white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive
UVC_WB_TEMP = 4250
# sharpness 0x0098091b (int)    : min=0 max=6 step=1 default=3 value=3
UVC_SHARPNESS = 100
# backlight_compensation 0x0098091c (int)    : min=0 max=2 step=1 default=1 value=1
UVC_BACKLIT_COMP = 1
# exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=157 flags=inactive
UVC_EXPOSURE_TIME = 157

# More UVC Video capture properties here:
# https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html
#
# Arducam wiki info site
# https://docs.arducam.com/UVC-Camera/Appilcation-Note/OpenCV-Python-GStreamer-on-linux/
#
# Setting Video formats using v4l2
# http://trac.gateworks.com/wiki/linux/v4l2
#
#


def setup_uvc_camera(device_index, size, fps):
    cap = cv2.VideoCapture(device_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


UVC_SETTINGS = {
    "brightness": UVC_BRIGHTNESS,
    "contrast": UVC_CONTRAST,
    "hue": UVC_HUE,
    "gamma": UVC_GAMMA,
    "gain": UVC_GAIN,
    "white_balance_temperature": UVC_WB_TEMP,
    "sharpness": UVC_SHARPNESS,
    "backlight_compensation": UVC_BACKLIT_COMP,
    "exposure_time_absolute": UVC_EXPOSURE_TIME,
}

# Set video format and size
cmd = f"v4l2-ctl --device {UVC_VIDEO_INDEX} --set-fmt-video=width={UVC_COLOR_SIZE[0]},height={UVC_COLOR_SIZE[1]}"
os.system(cmd)

# Set UVC SettingsNone
for k in list(UVC_SETTINGS.keys()):
    cmd = f"v4l2-ctl --device {UVC_VIDEO_INDEX} --set-ctrl={k}={UVC_SETTINGS[k]}"
    os.system(cmd)


class GripperImagePublisherNode(Node):
    def __init__(self):
        super().__init__("image_publisher_node")
        self.publisher = self.create_publisher(
            Image, "/gripper_camera/color/image_rect_raw", 15
        )
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(timer_period, self.timer_callback2)
        self.cv_bridge = CvBridge()
        self.uvc_camera = setup_uvc_camera(UVC_VIDEO_INDEX, UVC_COLOR_SIZE, UVC_FPS)
        self.image_msg = None
        # self.uvc_camera_thread = threading.Thread(target=self.stream_camera)
        # self.uvc_camera_thread.start()

    # def stream_camera(self):
    #     print("Starting Image Stream")
    #     while True:
    #     # Capture an image using OpenCV
    #         try:
    #             ret, image_uvc = self.uvc_camera.read()
    #             # Convert the OpenCV image to a ROS Image message
    #             self.image_msg = self.cv_bridge.cv2_to_imgmsg(image_uvc, encoding='bgr8')
    #             print("Updated")
    #         except Exception as e:
    #             print(f"Error UVC Cam: {e}")

    def timer_callback(self):
        # Publish the image message
        if self.image_msg is not None:
            # print("Published")
            self.publisher.publish(self.image_msg)

    def timer_callback2(self):
        try:
            ret, image_uvc = self.uvc_camera.read()
            # Convert the OpenCV image to a ROS Image message
            self.image_msg = self.cv_bridge.cv2_to_imgmsg(image_uvc, encoding="bgr8")
            # print("Updated")
        except Exception as e:
            print(f"Error UVC Cam: {e}")


def main(args=None):
    rclpy.init(args=args)

    image_publisher_node = GripperImagePublisherNode()

    executor = MultiThreadedExecutor()
    executor.add_node(image_publisher_node)
    try:
        rclpy.spin(image_publisher_node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
