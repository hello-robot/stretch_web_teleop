# Adapted from https://github.com/IntelRealSense/realsense-ros/tree/ros2-development/realsense2_camera/launch

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

json_path = os.path.join(
    get_package_share_directory("stretch_core"), "config", "HighAccuracyPreset.json"
)
D435_RESOLUTION = "424x240x15"
D405_RESOLUTION = "480x270x15"

# Starting with `realsense_ros` 4.55.1, the `.profile`` parameter is split by stream type.
# Here, we keep both the old and new parameters for backwards compatibility.
# https://github.com/IntelRealSense/realsense-ros/pull/3052
# fmt: off
base_configurable_parameters = [
    {'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
    {'name': 'camera_namespace',             'default': '', 'description': 'namespace for camera'},
    {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
    {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
    {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
    {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
    {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
    {'name': 'initial_reset',                'default': 'false', 'description': "''"},
    {'name': 'accelerate_gpu_with_glsl',     'default': "false", 'description': 'enable GPU acceleration with GLSL'},
    {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},  # noqa: E501
    {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},  # noqa: E501
    {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
    {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
    {'name': 'rgb_camera.profile',           'default': '0,0,0', 'description': 'color stream profile'},
    {'name': 'rgb_camera.color_profile',     'default': '0,0,0', 'description': 'color stream profile'},
    {'name': 'rgb_camera.color_format',      'default': 'RGB8', 'description': 'color stream format'},
    {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},  # noqa: E501
    {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
    {'name': 'enable_infra',                 'default': 'false', 'description': 'enable infra0 stream'},
    {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
    {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
    {'name': 'infra_rgb',                    'default': 'false', 'description': ''},
    {'name': 'enable_confidence',            'default': 'false', 'description': ''},
    {'name': 'depth_module.profile',         'default': '0,0,0', 'description': 'depth stream profile'},
    {'name': 'depth_module.depth_profile',   'default': '0,0,0', 'description': 'depth stream profile'},
    {'name': 'depth_module.depth_format',    'default': 'Z16', 'description': 'depth stream format'},
    {'name': 'depth_module.infra_profile',   'default': '0,0,0', 'description': 'infra streams (0/1/2) profile'},
    {'name': 'depth_module.infra_format',    'default': 'RGB8', 'description': 'infra0 stream format'},
    {'name': 'depth_module.infra1_format',   'default': 'Y8', 'description': 'infra1 stream format'},
    {'name': 'depth_module.infra2_format',   'default': 'Y8', 'description': 'infra2 stream format'},
    {'name': 'depth_module.color_profile',   'default': '0,0,0', 'description': 'infra2 stream format'},
    {'name': 'depth_module.exposure',        'default': '8500', 'description': 'Depth module manual exposure value'},
    {'name': 'depth_module.gain',            'default': '16', 'description': 'Depth module manual gain value'},
    {'name': 'depth_module.hdr_enabled',     'default': 'false', 'description': 'Depth module hdr enablement flag. Used for hdr_merge filter'},  # noqa: E501
    {'name': 'depth_module.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},  # noqa: E501
    {'name': 'depth_module.exposure.1',      'default': '7500', 'description': 'Depth module first exposure value. Used for hdr_merge filter'},  # noqa: E501
    {'name': 'depth_module.gain.1',          'default': '16', 'description': 'Depth module first gain value. Used for hdr_merge filter'},  # noqa: E501
    {'name': 'depth_module.exposure.2',      'default': '1', 'description': 'Depth module second exposure value. Used for hdr_merge filter'},  # noqa: E501
    {'name': 'depth_module.gain.2',          'default': '16', 'description': 'Depth module second gain value. Used for hdr_merge filter'},  # noqa: E501
    {'name': 'enable_sync',                  'default': 'false', 'description': "'enable sync mode'"},
    {'name': 'enable_rgbd',                  'default': 'false', 'description': "'enable rgbd topic'"},
    {'name': 'enable_gyro',                  'default': 'false', 'description': "'enable gyro stream'"},
    {'name': 'enable_accel',                 'default': 'false', 'description': "'enable accel stream'"},
    {'name': 'gyro_fps',                     'default': '0', 'description': "''"},
    {'name': 'accel_fps',                    'default': '0', 'description': "''"},
    {'name': 'unite_imu_method',             'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
    {'name': 'clip_distance',                'default': '-2.', 'description': "''"},
    {'name': 'angular_velocity_cov',         'default': '0.01', 'description': "''"},
    {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},
    {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},  # noqa: E501
    {'name': 'publish_tf',                   'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},  # noqa: E501
    {'name': 'tf_publish_rate',              'default': '0.0', 'description': '[double] rate in Hz for publishing dynamic TF'},  # noqa: E501
    {'name': 'pointcloud.enable',            'default': 'false', 'description': ''},
    {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
    {'name': 'pointcloud.stream_index_filter', 'default': '0', 'description': 'texture stream index for pointcloud'},
    {'name': 'pointcloud.ordered_pc',        'default': 'false', 'description': ''},
    {'name': 'pointcloud.allow_no_texture_points', 'default': 'false', 'description': "''"},
    {'name': 'align_depth.enable',           'default': 'false', 'description': 'enable align depth filter'},
    {'name': 'colorizer.enable',             'default': 'false', 'description': 'enable colorizer filter'},
    {'name': 'decimation_filter.enable',     'default': 'false', 'description': 'enable_decimation_filter'},
    {'name': 'spatial_filter.enable',        'default': 'false', 'description': 'enable_spatial_filter'},
    {'name': 'temporal_filter.enable',       'default': 'false', 'description': 'enable_temporal_filter'},
    {'name': 'disparity_filter.enable',      'default': 'false', 'description': 'enable_disparity_filter'},
    {'name': 'hole_filling_filter.enable',   'default': 'false', 'description': 'enable_hole_filling_filter'},
    {'name': 'hdr_merge.enable',             'default': 'false', 'description': 'hdr_merge filter enablement flag'},
    {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},  # noqa: E501
    {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consecutive reconnection attempts'},  # noqa: E501
    {'name': 'camera.aligned_depth_to_color.image_raw.png_level', 'default': '1', 'description': ''},
    {'name': 'gripper_camera.aligned_depth_to_color.image_raw.png_level', 'default': '1', 'description': ''},
    {'name': 'color_qos', 'default': 'SENSOR_DATA', 'description': ''},
    {'name': 'depth_qos', 'default': 'SENSOR_DATA', 'description': ''},
    {'name': 'infra_qos', 'default': 'SENSOR_DATA', 'description': ''},
]
d435_parameter_overrides = {
    'camera_name': 'camera',
    'device_type': 'd435',
    'json_file_path': json_path,
    'depth_module.profile': D435_RESOLUTION,
    'depth_module.depth_profile': D435_RESOLUTION,
    'depth_module.infra_profile': D435_RESOLUTION,
    'rgb_camera.profile': D435_RESOLUTION,
    'rgb_camera.color_profile': D435_RESOLUTION,
    'pointcloud.enable': 'true',
    'pointcloud.stream_filter': '',
    'pointcloud.stream_index_filter': '',
    'enable_sync': 'true',
    'align_depth.enable': 'false',
    'initial_reset': 'true',
    'allow_no_texture_points': 'true',
}
d405_parameter_overrides = {
    'camera_name': 'gripper_camera',
    'device_type': 'd405',
    'json_file_path': json_path,
    'depth_module.profile': D405_RESOLUTION,
    'depth_module.depth_profile': D405_RESOLUTION,
    'depth_module.infra_profile': D405_RESOLUTION,
    'rgb_camera.profile': D405_RESOLUTION,
    'depth_module.color_profile': D405_RESOLUTION,
    'pointcloud.enable': 'true',
    'pointcloud.stream_filter': '',
    'pointcloud.stream_index_filter': '',
    'enable_sync': 'true',
    'align_depth.enable': 'true',
    'initial_reset': 'true',
    'allow_no_texture_points': 'true',
}
# fmt: on


def apply_parameter_overrides(base_parameters, parameter_overrides):
    params = []
    for param in base_parameters:
        name = param["name"]
        default = param["default"]
        description = param["description"]
        params.append(
            {
                "name": name,
                "default": parameter_overrides[name]
                if name in parameter_overrides
                else default,
                "description": description,
            }
        )
    return params


def set_configurable_parameters(local_params):
    return dict(
        [
            (param["original_name"], LaunchConfiguration(param["name"]))
            for param in local_params
        ]
    )


def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default"],
            description=param["description"],
        )
        for param in parameters
    ]


def append_to_parameter_names(params, suffix):
    for param in params:
        param["original_name"] = param["name"]
        param["name"] += suffix


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def launch_setup(context, params, param_name_suffix="", remappings=[]):
    _config_file = LaunchConfiguration("config_file" + param_name_suffix).perform(
        context
    )
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)

    _output = LaunchConfiguration("output" + param_name_suffix)

    return [
        Node(
            package="realsense2_camera",
            namespace=LaunchConfiguration("camera_namespace" + param_name_suffix),
            name=LaunchConfiguration("camera_name" + param_name_suffix),
            executable="realsense2_camera_node",
            parameters=[params, params_from_file],
            output=_output,
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("log_level" + param_name_suffix),
            ],
            emulate_tty=True,
            remappings=remappings,
        )
    ]


def generate_launch_description():
    # Apply camera-specific parameter overrides
    d435_params = apply_parameter_overrides(
        base_configurable_parameters, d435_parameter_overrides
    )
    d405_params = apply_parameter_overrides(
        base_configurable_parameters, d405_parameter_overrides
    )

    # Add the depth compression level parameter
    d435_params.append(
        {
            "name": f"{d435_parameter_overrides['camera_name']}.aligned_depth_to_color.image_raw.png_level",
            "default": "1",  # PNG level 1 is the fastest
            "description": "Compression level for aligned depth",
        }
    )
    d405_params.append(
        {
            "name": f"{d405_parameter_overrides['camera_name']}.aligned_depth_to_color.image_raw.png_level",
            "default": "1",  # PNG level 1 is the fastest
            "description": "Compression level for aligned depth",
        }
    )

    # Add RGB compression parameters
    d435_params.append(
        {
            "name": f"{d435_parameter_overrides['camera_name']}.color.image_rect_raw.png_level",
            "default": "1",
            "description": "Compression level for the color stream",
        }
    )
    d405_params.append(
        {
            "name": f"{d405_parameter_overrides['camera_name']}.color.image_rect_raw.png_level",
            "default": "1",
            "description": "Compression level for the color stream",
        }
    )

    # Map each camera to a suffix
    params_to_suffix = [
        (d435_params, "1", []),
        (
            d405_params,
            "2",
            [
                ("/gripper_camera/color/image_rect_raw", "/gripper_camera/image_raw"),
                (
                    "/gripper_camera/color/image_rect_raw/compressed",
                    "/gripper_camera/image_raw/compressed",
                ),
            ],
        ),
    ]

    # Create the launch description
    ld = []
    for params, suffix, remappings in params_to_suffix:
        # Append the suffix to the parameter names
        append_to_parameter_names(params, suffix)

        # Declare the parameters
        ld += declare_configurable_parameters(params)

        # Create the node for these params
        ld.append(
            OpaqueFunction(
                function=launch_setup,
                kwargs={
                    "params": set_configurable_parameters(params),
                    "param_name_suffix": suffix,
                    "remappings": remappings,
                },
            )
        )

    # Add the D435i accel correction node
    ld.append(
        Node(
            package="stretch_core",
            executable="d435i_accel_correction",
            output="screen",
        )
    )

    return LaunchDescription(ld)
