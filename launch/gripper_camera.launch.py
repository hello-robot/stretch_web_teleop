import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='gripper_camera',
            output='screen',
            parameters=[
                {'video_device': '/dev/hello-gripper-camera'},
                {'image_width': 1024},
                {'image_height': 768},
                {'framerate': 15.0},
                {'pixel_format': 'yuyv'},
                {'brightness': -40},
                {'contrast': 40},
                {'saturation': 60},
                {'hue': 0},
                {'sharpness': 100},
                {'autoexposure': True},
                {'exposure': 150},
                {'auto_white_balance': False},
                {'white_balance': 4250},
                {'gain': 80},
                {'camera_frame_id': 'gripper_camera'},
                {'camera_name': 'gripper_camera'},
                {'io_method': 'mmap'}
            ],
            remappings=[('/usb_cam/image_raw', '/gripper_camera/image_raw')]
        ),
        # You can add more nodes or configurations here if needed.
    ])