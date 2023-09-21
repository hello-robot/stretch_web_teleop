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
            name='navigation_camera',
            output='screen',
            parameters=[
                {'video_device': '/dev/hello-navigation-camera'},
                {'image_width': 1024},
                {'image_height': 768},
                {'framerate': 30.0},
                {'pixel_format': 'yuyv'},
                {'brightness': -40},
                {'contrast': 40},
                {'hue': 0},
                {'saturation': 60},
                {'sharpness': 100},
                {'autoexposure': True},
                {'exposure': 150},
                {'auto_white_balance': False},
                {'white_balance': 4250},
                {'gain': 50},
                {'camera_frame_id': 'navigation_camera'},
                {'camera_name': 'navigation_camera'},
                {'io_method': 'mmap'}
            ],
            remappings=[('/image_raw', '/navigation_camera/image_raw')]
        ),
        # You can add more nodes or configurations here if needed.
    ])