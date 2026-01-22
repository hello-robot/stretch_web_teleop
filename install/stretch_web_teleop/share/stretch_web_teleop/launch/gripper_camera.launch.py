from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="gripper_camera",
                output="screen",
                parameters=[
                    {"video_device": "/dev/hello-gripper-camera"},
                    {"image_width": 1024},
                    {"image_height": 768},
                    {"framerate": 30.0},
                    {"pixel_format": "yuyv"},
                    {"brightness": -40},
                    {"contrast": 40},
                    {"saturation": 60},
                    {"hue": 0},
                    {"sharpness": 100},
                    {"autoexposure": True},
                    {"exposure": 150},
                    {"auto_white_balance": False},
                    {"white_balance": 4250},
                    {"gain": 80},
                    {"camera_frame_id": "gripper_camera"},
                    {"camera_name": "gripper_camera"},
                    {"io_method": "mmap"},
                ],
                remappings=[
                    ("/image_raw", "/gripper_camera/image_raw")
                    # usb_cam does not publish compressed images:
                    # https://github.com/ros-drivers/usb_cam/blob/52dd75fb78ae608a2c40eea60f6e8eac673b91e8/src/ros2/usb_cam_node.cpp#L50
                    # ("/image_raw/compressed", "/gripper_camera/image_raw/compressed")
                ],
            ),
            # You can add more nodes or configurations here if needed.
        ]
    )
