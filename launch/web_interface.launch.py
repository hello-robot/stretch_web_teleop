from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    teleop_interface_package = str(get_package_share_path('stretch_teleop_interface'))
    core_package = str(get_package_share_path('stretch_core'))
    rosbridge_package = str(get_package_share_path('rosbridge_server'))
    
    # Declare launch arguments
    params_file = DeclareLaunchArgument('params', default_value=[
        PathJoinSubstitution([teleop_interface_package, 'config', 'configure_video_streams_params.yaml'])])
    # map_yaml = DeclareLaunchArgument('map_yaml', description='filepath to previously captured map (required)')
    gripper_camera_arg = DeclareLaunchArgument('gripper_camera', default_value='false')
    navigation_camera_arg = DeclareLaunchArgument('navigation_camera', default_value='false')
    certfile_arg = DeclareLaunchArgument('certfile', default_value='your_certfile.pem')
    keyfile_arg = DeclareLaunchArgument('keyfile', default_value='your_keyfile.pem')

    # Realsense D435i
    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([core_package, 'launch', 'd435i_low_resolution.launch.py']))
    )

    # Gripper Camera Group
    gripper_camera_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('gripper_camera')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([teleop_interface_package, 'launch', 'gripper_camera.launch.py']))
            )
        ]
    )

    # Gripper Camera Node
    gripper_camera_node = Node(
        package='image_publisher',
        executable='image_publisher_node',
        name='gripper_camera',
        output='screen',
        parameters=[{'frame_id': 'gripper_camera', 'publish_rate': 15.0}],
        arguments=[PathJoinSubstitution([teleop_interface_package, 'nodes', 'blank_image.png'])],
        condition=UnlessCondition(LaunchConfiguration('gripper_camera'))
    )

    # Navigation Camera Group
    navigation_camera_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('navigation_camera')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([teleop_interface_package, 'launch', 'navigation_camera.launch.py']))
            )
        ]
    )

    # Navigation Camera Node
    navigation_camera_node = Node(
        package='image_publisher',
        executable='image_publisher_node',
        name='navigation_camera',
        output='screen',
        parameters=[{'frame_id': 'navigation_camera', 'publish_rate': 15.0}],
        arguments=[PathJoinSubstitution([teleop_interface_package, 'nodes', 'blank_image.png'])],
        condition=UnlessCondition(LaunchConfiguration('navigation_camera'))
    )

    # Stretch Driver
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([core_package, 'launch', 'stretch_driver.launch.py'])),
        launch_arguments={'broadcast_odom_tf': 'True'}.items())
    
    # Rosbridge Websocket
    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(PathJoinSubstitution([rosbridge_package, 'launch', 'rosbridge_websocket_launch.xml'])),
        launch_arguments={
            'port': '9090',
            'address': 'localhost',
            'ssl': 'true',
            'certfile': PathJoinSubstitution([teleop_interface_package, 'certificates', LaunchConfiguration('certfile')]),
            'keyfile': PathJoinSubstitution([teleop_interface_package, 'certificates', LaunchConfiguration('keyfile')]),
            'authenticate': 'false',
        }.items()
    )

    # Configure Video Streams
    configure_video_streams_node = Node(
        package='stretch_teleop_interface',
        executable='configure_video_streams.py',
        name='configure_video_streams',
        output='screen',
        arguments=[LaunchConfiguration('params')]
    )

    return LaunchDescription([
        params_file,
        # map_yaml,
        gripper_camera_arg,
        navigation_camera_arg,
        certfile_arg,
        keyfile_arg,
        d435i_launch,
        gripper_camera_group,
        gripper_camera_node,
        navigation_camera_group,
        navigation_camera_node,
        stretch_driver_launch,
        rosbridge_launch,
        configure_video_streams_node,
    ])
