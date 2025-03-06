# Copyright 2023 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import Node

# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(
    get_package_share_directory('whill_navi2'),
    'config',
    'params',
    'zed_camera_params.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('whill_navi2'),
    'config',
    'urdf',
    'zed_descr.urdf.xacro'
)

def launch_setup(context, *args, **kwargs):
    wrapper_dir = get_package_share_directory('zed_wrapper')

    # Launch configuration variables
    svo_path = LaunchConfiguration('svo_path')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    sim_address = LaunchConfiguration('sim_address')
    sim_port = LaunchConfiguration('sim_port')

    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    zed_node_name = LaunchConfiguration('zed_node_name')

    config_common_path = LaunchConfiguration('config_path')

    serial_number = LaunchConfiguration('serial_number')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path = LaunchConfiguration('xacro_path')

    ros_params_override_path = LaunchConfiguration('ros_params_override_path')

    gnss_frame = LaunchConfiguration('gnss_frame')

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    zed_node_name_val = zed_node_name.perform(context)

    if (camera_name_val == ''):
        camera_name_val = 'zed2i'

    config_camera_path = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config',
        'params',
        camera_model_val + '_params.yaml'
    )

    # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=camera_name_val,
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_name_val, ' ',
                    'camera_model:=', camera_model_val, ' '
                ])
        }]
    )

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package='zed_wrapper',
        namespace=camera_name_val,
        executable='zed_wrapper',
        name=zed_node_name,
        output='screen',
        # prefix=['xterm -e valgrind --tools=callgrind'],
        # prefix=['xterm -e gdb -ex run --args'],
        #prefix=['gdbserver localhost:3000'],
        parameters=[
            # YAML files
            config_common_path,  # Common parameters
            config_camera_path,  # Camera related parameters
            # Overriding
            {
                'use_sim_time': use_sim_time,
                'simulation.sim_enabled': sim_mode,
                'simulation.sim_address': sim_address,
                'simulation.sim_port': sim_port,
                'general.camera_name': camera_name_val,
                'general.camera_model': camera_model_val,
                'general.svo_file': svo_path,
                'general.serial_number': serial_number,
                'pos_tracking.publish_tf': publish_tf,
                'pos_tracking.publish_map_tf': publish_map_tf,
                'sensors.publish_imu_tf': publish_imu_tf
            },
            ros_params_override_path,
        ]
    )
    
    # Publish Imu Data with enu frame
    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[{
            "publish_tf" : False,
            "use_mag" : True,
            "mag_bias_x" : -2.742024e-7,
            "mag_bias_y" : 2.073313e-7,
            "mag_bias_z" : -6.244913e-7,
            "orientation_stddev" : 0.009995,
            "world_frame" : "nwu"
        }],
        remappings=[
            ("imu/data_raw", "{}/{}/imu/data".format(camera_name_val, zed_node_name_val)),
            ("imu/data", "{}/{}/imu/data_enu".format(camera_name_val, zed_node_name_val)),
            ("imu/mag", "{}/{}/imu/mag".format(camera_name_val, zed_node_name_val))
        ]
    )

    return [
        rsp_node,
        zed_wrapper_node,
        imu_filter_node
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm'],
                default_value=TextSubstitution(text='zed2i')),
            DeclareLaunchArgument(
                'zed_node_name',
                default_value='zed_node',
                description='The name of the zed_wrapper node. All the topic will have the same prefix: `/<camera_name>/<zed_node_name>/`'),
            DeclareLaunchArgument(
                'config_path',
                default_value=TextSubstitution(text=default_config_common),
                description='Path to the YAML configuration file for the camera.'),
            DeclareLaunchArgument(
                'serial_number',
                default_value='0',
                description='The serial number of the camera to be opened. It is mandatory to use this parameter in multi-camera rigs to distinguish between different cameras.'),
            DeclareLaunchArgument(
                'publish_urdf',
                default_value='true',
                description='Enable URDF processing and starts Robot State Published to propagate static TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='false',
                description='Enable publication of the `odom -> camera_link` TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_map_tf',
                default_value='false',
                description='Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_imu_tf',
                default_value='true',
                description='Enable publication of the IMU TF. Note: Ignored if `publish_tf` is False.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'xacro_path',
                default_value=TextSubstitution(text=default_xacro_path),
                description='Path to the camera URDF file as a xacro file.'),
            DeclareLaunchArgument(
                'ros_params_override_path',
                default_value='',
                description='The path to an additional parameters file to override the defaults'),
            DeclareLaunchArgument(
                'svo_path',
                default_value=TextSubstitution(text='live'),
                description='Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`.'),
            DeclareLaunchArgument(
                'gnss_frame',
                default_value='',
                description='Name of the GNSS link frame. Leave empty if not used. Remember to set the transform `camera_link` -> `gnss_frame` in the URDF file.'),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Enable simulation time mode.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'sim_mode',
                default_value='false',
                description='Enable simulation mode. Set `sim_address` and `sim_port` to configure the simulator input.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'sim_address',
                default_value='127.0.0.1',
                description='The connection address of the simulation server. See the documentation of the supported simulation plugins for more information.'),
            DeclareLaunchArgument(
                'sim_port',
                default_value='30000',
                description='The connection port of the simulation server. See the documentation of the supported simulation plugins for more information.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
