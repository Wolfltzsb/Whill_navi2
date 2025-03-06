#!/usr/bin/env python

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch_ros.actions import Node
from launch.event_handlers import OnExecutionComplete, OnProcessStart, OnShutdown, OnProcessExit

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from whill_navi2.modules.ros2_launch_utils import get_param_dict, os, LogInfo, ExecuteProcess, FindExecutable
from whill_navi2.modules.ros2_launch_utils import get_file_path

def generate_launch_description():

    ld = LaunchDescription()
    
    config_path = get_file_path("whill_navi2", "m20_lidar_node_params.yaml")
    config_dict = get_param_dict(config_path, "lumotive_driver")
    
    drv_node = Node(
        package='ylm_ros2',
        executable='lumotive_driver',
        name='lumotive_driver',
        output='screen',
        parameters=[config_path]
    )
    ld.add_action(drv_node)
    
    start_scan = RegisterEventHandler(
        OnProcessStart(
            target_action=drv_node,
            on_start=[
                Node(
                    package="whill_navi2",
                    executable="m20_com_node",
                    parameters=[{
                        "ip_address" : config_dict["sensor_ip"],
                        "mode" : "auto",
                        "m20_lidar_node_name" : 'lumotive_driver',
                        "angle_range": [-5, 25]
                    }]
                )
            ]
        )
    )
    ld.add_action(start_scan)
        
    return ld