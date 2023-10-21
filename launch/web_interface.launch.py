import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    teleop_interface_package = str(get_package_share_path('stretch_teleop_interface'))
    core_package = str(get_package_share_path('stretch_core'))
    rosbridge_package = str(get_package_share_path('rosbridge_server'))
    stretch_core_path = str(get_package_share_directory('stretch_core'))
    stretch_navigation_path = str(get_package_share_directory('stretch_nav2'))
    navigation_bringup_path = str(get_package_share_directory('nav2_bringup'))
    
    # Declare launch arguments
    params_file = DeclareLaunchArgument('params', default_value=[
        PathJoinSubstitution([teleop_interface_package, 'config', 'configure_video_streams_params.yaml'])])
    map_yaml = DeclareLaunchArgument('map_yaml', description='filepath to previously captured map (required)')
    gripper_camera_arg = DeclareLaunchArgument('gripper_camera', default_value='false')
    navigation_camera_arg = DeclareLaunchArgument('navigation_camera', default_value='false')
    certfile_arg = DeclareLaunchArgument('certfile', default_value='your_certfile.pem')
    keyfile_arg = DeclareLaunchArgument('keyfile', default_value='your_keyfile.pem')
    nav2_params_file_param = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(stretch_navigation_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    dict_file_path = os.path.join(core_package, 'config', 'stretch_marker_dict.yaml')
    depthimage_to_laserscan_config = os.path.join(core_package, 'config', 'depthimage_to_laser_scan_params.yaml')

    # Realsense D435i
    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([core_package, 'launch', 'd435i_low_resolution.launch.py']))
    )

    # Gripper Camera Group
    gripper_camera_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('gripper_camera')),
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
        name='gripper_camera_node',
        output='screen',
        parameters=[{'frame_id': 'gripper_camera', 'publish_rate': 15.0}],
        arguments=[PathJoinSubstitution([teleop_interface_package, 'nodes', 'blank_image.png'])],
        condition=UnlessCondition(LaunchConfiguration('gripper_camera'))
    )

    # Navigation Camera Group
    navigation_camera_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('navigation_camera')),
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
        name='navigation_camera_node',
        output='screen',
        parameters=[{'frame_id': 'navigation_camera', 'publish_rate': 15.0}],
        arguments=[PathJoinSubstitution([teleop_interface_package, 'nodes', 'blank_image.png'])],
        condition=UnlessCondition(LaunchConfiguration('navigation_camera'))
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': PathJoinSubstitution([teleop_interface_package, 'maps', LaunchConfiguration('map_yaml')])}]
    )

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
    )
                        
    tf2_web_republisher_node = Node(
        package='tf2_web_republisher_py',
        executable='tf2_web_republisher',
        name='tf2_web_republisher_node'
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
        # name='configure_video_streams_node',
        output='screen',
        arguments=[LaunchConfiguration('params')]
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    navigation_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/bringup_launch.py']),
        launch_arguments={'use_sim_time': 'false', 
                          'autostart': 'true',
                          'map': PathJoinSubstitution([teleop_interface_package, 'maps', LaunchConfiguration('map_yaml')]),
                          'params_file': LaunchConfiguration('nav2_params_file'),
                          'use_rviz': 'false'}.items())

    detect_aruco_markers_node = Node(
        package='stretch_teleop_interface',
        executable='detect_aruco_markers.py',
        name='detect_aruco_node',
        output='screen',
        parameters=[dict_file_path]
    )

    navigate_to_aruco_node = Node(
        package='stretch_teleop_interface',
        executable='navigate_to_aruco',
        name='navigate_to_aruco_node',
        output='screen',
        parameters=[dict_file_path]
    )

    head_scan_node = Node(
        package='stretch_teleop_interface',
        executable='head_scan.py',
        name='head_scan_node',
        output='screen'
    )

    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[('depth', '/camera/depth/image_rect_raw'),
                    ('depth_camera_info', '/camera/depth/camera_info')],
        parameters=[depthimage_to_laserscan_config]
    )

    ld = LaunchDescription([
        params_file,
        map_yaml,
        nav2_params_file_param,
        gripper_camera_arg,
        navigation_camera_arg,
        certfile_arg,
        keyfile_arg,
        d435i_launch,
        gripper_camera_group,
        gripper_camera_node,
        navigation_camera_group,
        navigation_camera_node,
        configure_video_streams_node,
        # map_server_cmd,
        # start_lifecycle_manager_cmd,
        tf2_web_republisher_node,
        stretch_driver_launch,
        rosbridge_launch,
        rplidar_launch,
        navigation_bringup_launch,
        # detect_aruco_markers_node,
        # head_scan_node,
        # depthimage_to_laserscan_node
    ])

    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " service call ",
                    "/reinitialize_global_localization ",
                    "std_srvs/srv/Empty ",
                    "\"{}\"",
                ]
            ],
            shell=True,
        )
    )

    return ld
