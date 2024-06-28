import fnmatch
import os

import stretch_body.robot_params
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def symlinks_to_has_beta_teleop_kit():
    usb_device_seen = {
        "hello-navigation-camera": False,
        "hello-gripper-camera": False,
    }

    listOfFiles = os.listdir("/dev")
    pattern = "hello*"
    for entry in listOfFiles:
        if fnmatch.fnmatch(entry, pattern):
            usb_device_seen[entry] = True

    return all(usb_device_seen.values())


def symlinks_to_has_nav_head_cam():
    usb_device_seen = {
        "hello-nav-head-camera": False,
    }

    listOfFiles = os.listdir("/dev")
    pattern = "hello*"
    for entry in listOfFiles:
        if fnmatch.fnmatch(entry, pattern):
            usb_device_seen[entry] = True

    return all(usb_device_seen.values())


def map_configuration_to_drivers(model, tool, has_beta_teleop_kit, has_nav_head_cam):
    """This method maps configurations to drivers. I.e. it identifies the robot configuration
    based on the variables provided and returns which drivers should be activated. If the
    variables don't constitute a valid configuration, something is wrong with the hardware,
    so the function raises an exception.

    Returns
    -------
    Tuple
        tuple with four elements:
          which_realsense_drivers ('d435i-only' or 'both'),
          add_gripper_driver (True or False),
          add_navigation_driver (True or False),
          add_head_nav_driver (True or False)
    """
    # Stretch RE1
    if (
        model == "RE1V0"
        and tool == "tool_stretch_gripper"
        and has_beta_teleop_kit is False
        and has_nav_head_cam is False
    ):
        return "d435-only", False, False, False
    elif (
        model == "RE1V0"
        and tool == "tool_stretch_gripper"
        and has_beta_teleop_kit is True
        and has_nav_head_cam is False
    ):
        return "d435-only", True, True, False
    elif (
        model == "RE1V0"
        and tool == "tool_stretch_dex_wrist"
        and has_beta_teleop_kit is False
        and has_nav_head_cam is False
    ):
        return "d435-only", False, False, False
    elif (
        model == "RE1V0"
        and tool == "tool_stretch_dex_wrist"
        and has_beta_teleop_kit is True
        and has_nav_head_cam is False
    ):
        return "d435-only", True, True, False
    # Stretch 2
    elif (
        model == "RE2V0"
        and tool == "tool_stretch_gripper"
        and has_beta_teleop_kit is False
        and has_nav_head_cam is False
    ):
        return "d435-only", False, False, False
    elif (
        model == "RE2V0"
        and tool == "tool_stretch_gripper"
        and has_beta_teleop_kit is True
        and has_nav_head_cam is False
    ):
        return "d435-only", True, True, False
    elif (
        model == "RE2V0"
        and tool == "tool_stretch_dex_wrist"
        and has_beta_teleop_kit is False
        and has_nav_head_cam is False
    ):
        return "d435-only", False, False, False
    elif (
        model == "RE2V0"
        and tool == "tool_stretch_dex_wrist"
        and has_beta_teleop_kit is True
        and has_nav_head_cam is False
    ):
        return "d435-only", True, True, False
    # Stretch 2+ (upgraded Stretch 2)
    elif (
        model == "RE2V0"
        and tool == "eoa_wrist_dw3_tool_sg3"
        and has_beta_teleop_kit is False
        and has_nav_head_cam is True
    ):
        return "both", False, False, True
    elif (
        model == "RE2V0"
        and tool == "eoa_wrist_dw3_tool_nil"
        and has_beta_teleop_kit is False
        and has_nav_head_cam is True
    ):
        return "both", False, False, True
    # Stretch 3
    elif (
        model == "SE3"
        and tool == "eoa_wrist_dw3_tool_sg3"
        and has_beta_teleop_kit is False
        and has_nav_head_cam is True
    ):
        return "both", False, False, True
    elif (
        model == "SE3"
        and tool == "eoa_wrist_dw3_tool_nil"
        and has_beta_teleop_kit is False
        and has_nav_head_cam is True
    ):
        return "both", False, False, True
    elif (
        model == "SE3"
        and tool == "eoa_wrist_dw3_tool_tablet_12in"
        and has_beta_teleop_kit is False
        and has_nav_head_cam is True
    ):
        return "both", False, False, True

    raise ValueError(
        f"cannot find valid configuration for model={model}, tool={tool}, "
        f"has_beta_teleop_kit={has_beta_teleop_kit}, has_nav_head_cam={has_nav_head_cam}"
    )


def generate_launch_description():
    teleop_interface_package = str(get_package_share_path("stretch_web_teleop"))
    core_package = str(get_package_share_path("stretch_core"))
    rosbridge_package = str(get_package_share_path("rosbridge_server"))
    stretch_core_path = str(get_package_share_directory("stretch_core"))
    stretch_navigation_path = str(get_package_share_directory("stretch_nav2"))

    _, robot_params = stretch_body.robot_params.RobotParams().get_params()
    stretch_serial_no = robot_params["robot"]["serial_no"]
    stretch_model = robot_params["robot"]["model_name"]
    stretch_tool = robot_params["robot"]["tool"]
    stretch_has_beta_teleop_kit = symlinks_to_has_beta_teleop_kit()
    stretch_has_nav_head_cam = symlinks_to_has_nav_head_cam()
    (
        drivers_realsense,
        driver_gripper_cam,
        driver_navigation_cam,
        driver_nav_head_cam,
    ) = map_configuration_to_drivers(
        stretch_model,
        stretch_tool,
        stretch_has_beta_teleop_kit,
        stretch_has_nav_head_cam,
    )

    # Declare launch arguments
    params_file = DeclareLaunchArgument(
        "params",
        default_value=[
            PathJoinSubstitution(
                [
                    teleop_interface_package,
                    "config",
                    "configure_video_streams_params.yaml",
                ]
            )
        ],
    )
    map_yaml = DeclareLaunchArgument(
        "map_yaml", description="filepath to previously captured map", default_value=""
    )
    certfile_arg = DeclareLaunchArgument(
        "certfile", default_value=stretch_serial_no + "+6.pem"
    )
    keyfile_arg = DeclareLaunchArgument(
        "keyfile", default_value=stretch_serial_no + "+6-key.pem"
    )
    nav2_params_file_param = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(
            stretch_navigation_path, "config", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    # Start collecting nodes to launch
    ld = LaunchDescription(
        [
            map_yaml,
            nav2_params_file_param,
            params_file,
            certfile_arg,
            keyfile_arg,
        ]
    )

    if drivers_realsense == "d435-only":
        # Launch only D435i if there is no D405
        ld.add_action(
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    core_package,
                                    "launch",
                                    "d435i_low_resolution.launch.py",
                                ]
                            )
                        )
                    )
                ]
            )
        )
    elif drivers_realsense == "both":
        # Launch both D435i and D405
        ld.add_action(
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    teleop_interface_package,
                                    "launch",
                                    "multi_camera.launch.py",
                                ]
                            )
                        )
                    )
                ]
            )
        )

    if driver_navigation_cam is True:
        # Beta Teleop Kit Navigation Camera
        ld.add_action(
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    core_package,
                                    "launch",
                                    "beta_navigation_camera.launch.py",
                                ]
                            )
                        )
                    )
                ]
            )
        )

    if driver_nav_head_cam is True:
        # Nav Head Wide Angle Camera
        ld.add_action(
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    teleop_interface_package,
                                    "launch",
                                    "navigation_camera.launch.py",
                                ]
                            )
                        )
                    )
                ]
            )
        )

    if driver_gripper_cam is True:
        # Beta Teleop Kit Gripper Camera
        ld.add_action(
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    core_package,
                                    "launch",
                                    "beta_gripper_camera.launch.py",
                                ]
                            )
                        )
                    )
                ]
            )
        )

    if driver_navigation_cam is False and driver_nav_head_cam is False:
        # Blank Navigation Camera Node
        # Publish blank image if no navigation camera exists
        ld.add_action(
            Node(
                package="image_publisher",
                executable="image_publisher_node",
                name="navigation_camera_node",
                output="screen",
                parameters=[{"publish_rate": 15.0}],
                remappings=[("image_raw", "/navigation_camera/image_raw")],
                arguments=[
                    PathJoinSubstitution(
                        [teleop_interface_package, "nodes", "blank_image.png"]
                    )
                ],
            )
        )

    if drivers_realsense == "d435-only" and driver_gripper_cam is False:
        # Blank Gripper Camera Node
        # Publish blank image if there is no gripper camera exists
        ld.add_action(
            Node(
                package="image_publisher",
                executable="image_publisher_node",
                name="gripper_camera_node",
                output="screen",
                parameters=[{"publish_rate": 15.0}],
                remappings=[("image_raw", "/gripper_camera/color/image_rect_raw")],
                arguments=[
                    PathJoinSubstitution(
                        [teleop_interface_package, "nodes", "blank_image.png"]
                    )
                ],
            )
        )

    tf2_web_republisher_node = Node(
        package="tf2_web_republisher_py",
        executable="tf2_web_republisher",
        name="tf2_web_republisher_node",
    )
    ld.add_action(tf2_web_republisher_node)

    # Stretch Driver
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([core_package, "launch", "stretch_driver.launch.py"])
        ),
        launch_arguments={"broadcast_odom_tf": "True"}.items(),
    )
    ld.add_action(stretch_driver_launch)

    # Rosbridge Websocket
    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution(
                [rosbridge_package, "launch", "rosbridge_websocket_launch.xml"]
            )
        ),
        launch_arguments={
            "port": "9090",
            "address": "localhost",
            "ssl": "true",
            "certfile": PathJoinSubstitution(
                [
                    teleop_interface_package,
                    "certificates",
                    LaunchConfiguration("certfile"),
                ]
            ),
            "keyfile": PathJoinSubstitution(
                [
                    teleop_interface_package,
                    "certificates",
                    LaunchConfiguration("keyfile"),
                ]
            ),
            "authenticate": "false",
        }.items(),
    )
    ld.add_action(rosbridge_launch)

    # Configure Video Streams
    labels = ["overhead", "realsense", "gripper"]
    for i in range(len(labels)):
        bools = ["False", "False", "False"]
        bools[i] = "True"
        label = labels[i]
        configure_video_streams_node = Node(
            package="stretch_web_teleop",
            executable="configure_video_streams.py",
            name=f"configure_video_streams_{label}",
            output="screen",
            arguments=[
                LaunchConfiguration("params"),
                str(stretch_has_beta_teleop_kit),
                *bools,
            ],
            parameters=[
                {
                    "has_beta_teleop_kit": stretch_has_beta_teleop_kit,
                    "stretch_tool": stretch_tool,
                }
            ],
        )
        ld.add_action(configure_video_streams_node)

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, "/launch/rplidar.launch.py"])
    )
    ld.add_action(rplidar_launch)

    navigation_bringup_launch = GroupAction(
        condition=LaunchConfigurationNotEquals("map_yaml", ""),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [stretch_navigation_path, "/launch/bringup_launch.py"]
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "autostart": "true",
                    "map": PathJoinSubstitution(
                        [
                            teleop_interface_package,
                            "maps",
                            LaunchConfiguration("map_yaml"),
                        ]
                    ),
                    "params_file": LaunchConfiguration("nav2_params_file"),
                    "use_rviz": "false",
                }.items(),
            )
        ],
    )
    ld.add_action(navigation_bringup_launch)

    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " service call ",
                    "/reinitialize_global_localization ",
                    "std_srvs/srv/Empty ",
                    '"{}"',
                ]
            ],
            shell=True,
        ),
    )

    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " param set ",
                    "/rosbridge_websocket ",
                    "std_msgs/msg/Bool ",
                    "true",
                ]
            ],
            shell=True,
        )
    )

    # ld.add_action(
    #     ExecuteProcess(
    #         cmd=[
    #             [
    #                 FindExecutable(name="ros2"),
    #                 " param set ",
    #                 "/gripper_camera ",
    #                 "depth_module.enable_auto_exposure ",
    #                 "true",
    #             ]
    #         ],
    #         shell=True,
    #     )
    # )

    # Move To Pre-grasp Action Server
    move_to_pregrasp_node = Node(
        package="stretch_web_teleop",
        executable="move_to_pregrasp.py",
        output="screen",
        arguments=[LaunchConfiguration("params")],
        parameters=[],
    )
    ld.add_action(move_to_pregrasp_node)

    return ld
