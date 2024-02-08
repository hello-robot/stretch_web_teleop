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
                {'image_width': 800},
                {'image_height': 600},
                {'framerate': 15.0},
                {'pixel_format': 'mjpeg2rgb'},
                {'brightness': 10},
                {'contrast': 30},
                {'hue': 0},
                {'saturation': 80},
                {'sharpness': 3},
                {'gamma': 80},
                {'exposure_auto': True},
                #{'exposure': 150},
                {'white_balance_temperature_auto': True},
                #{'white_balance': 4250},
                {'gain': 10},
                {'camera_frame_id': 'navigation_camera'},
                {'camera_name': 'navigation_camera'},
                {'io_method': 'mmap'}
            ],
            remappings=[('/image_raw', '/navigation_camera/image_raw')]
        ),
        # You can add more nodes or configurations here if needed.
    ])
