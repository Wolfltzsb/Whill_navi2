from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from whill_navi2.modules.ros2_launch_utils import IfCondition


def generate_launch_description():
    
    hokuyo_frame = "laser_front"
    velodyne_frame = "velodyne"
    adis_imu_frame = "imu_adis"
    wit_imu_frame = "imu_wit"
    ublox_frame = "ublox"
    whill_frame = "whill_link"
    zed_camera_frame = "zed2i_camera_link"
    m20_lidar_frame = "m20_lidar"
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(name="use_hokuyo_frame", default_value="false"))
    use_hokuyo_frame = LaunchConfiguration("use_hokuyo_frame")
    ld.add_action(DeclareLaunchArgument(name="use_velodyne_frame", default_value="false"))
    use_velodyne_frame = LaunchConfiguration("use_velodyne_frame")
    ld.add_action(DeclareLaunchArgument(name="use_adis_imu_frame", default_value="false"))
    use_adis_imu_frame = LaunchConfiguration("use_adis_imu_frame")
    ld.add_action(DeclareLaunchArgument(name="use_wit_imu_frame", default_value="false"))
    use_wit_imu_frame = LaunchConfiguration("use_wit_imu_frame")
    ld.add_action(DeclareLaunchArgument(name="use_ublox_frame", default_value="false"))
    use_ublox_frame = LaunchConfiguration("use_ublox_frame")
    ld.add_action(DeclareLaunchArgument(name="use_zed_camera_frame", default_value="false"))
    use_zed_camera_frame = LaunchConfiguration("use_zed_camera_frame")
    ld.add_action(DeclareLaunchArgument(name="use_m20_lidar_frame", default_value="false"))
    use_m20_lidar_frame = LaunchConfiguration("use_m20_lidar_frame")

         
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='utm30_to_base',
        arguments = [
            '--x', '0.50', '--y', '0.0', '--z', '0.33', 
            '--roll', '3.14159', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', "base_link", 
            '--child-frame-id', hokuyo_frame
        ],
        condition=IfCondition(use_hokuyo_frame)
    ))
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_to_base',
        arguments = [
            '--x', '0.0', '--y', '0.0', '--z', '1.115', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', "base_link", 
            '--child-frame-id', velodyne_frame, 
        ],
        condition=IfCondition(use_velodyne_frame)
    ))
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='adis_imu_to_base',
        arguments = [
            '--x', '0.045', '--y', '0.1', '--z', '0.532', 
            '--roll', '0.0', '--pitch', '-0.0', '--yaw', '0.0', 
            '--frame-id', "base_link", 
            '--child-frame-id', adis_imu_frame, 
        ],
        # IMU TF Arguments
        # arguments = [
        #     '--x', '0.44', '--y', '0.23', '--z', '0.2575', 
        #     '--roll', '-1.57079', '--pitch', '0.0', '--yaw', '0.0', 
        #     '--frame-id', 'base_link', '--child-frame-id', 'imu'
        # ]
        condition=IfCondition(use_adis_imu_frame)
    ))
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wit_imu_to_base',
        # arguments = [
        #     '--x', '0.44', '--y', '-0.23', '--z', '0.2575', 
        #     '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
        #     '--frame-id', base_frame_arg.default_value[0].perform('tf2_static_launch'), 
        #     '--child-frame-id', wit_imu_frame_arg.default_value[0].perform('tf2_static_launch'), 
        # ]
        arguments = [
            '--x', '0.40', '--y', '-0.19', '--z', '0.4275', 
            '--roll', '0.0', '--pitch', '-0.1047', '--yaw', '-1.5707963', 
            '--frame-id', "base_link", 
            '--child-frame-id', wit_imu_frame, 
        ],
        condition=IfCondition(use_wit_imu_frame)
    ))
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ublox_to_base',
        arguments = [
            '--x', '0.71', '--y', '0.0', '--z', '0.0175', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', "base_link", 
            '--child-frame-id', ublox_frame, 
        ],
        condition=IfCondition(use_ublox_frame)
    ))
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ylm20_to_base',
        arguments = [
            '--x', '0.43', '--y', '0.0', '--z', '0.44', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', "base_link", 
            '--child-frame-id', m20_lidar_frame, 
        ],
        condition=IfCondition(use_m20_lidar_frame)
    ))
    
    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed2i_to_base",
        arguments = [
            '--x', '0.43', '--y', '0.0', '--z', '0.365', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', "base_link", 
            '--child-frame-id', zed_camera_frame,             
        ],
        condition=IfCondition(use_zed_camera_frame)
    ))
    
    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="whill_to_base",
        arguments = [
            '--x', '0.0', '--y', '0.0', '--z', '0.0', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', "base_link", 
            '--child-frame-id', whill_frame,             
        ]
    ))
    
    # ld.add_action(Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="origin_to_map",
    #     arguments = [
    #         '--x', '0.0', '--y', '0.0', '--z', '0.0', 
    #         '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
    #         '--frame-id', "map", 
    #         '--child-frame-id', "origin",             
    #     ],
    #     condition=IfCondition(use_ublox_frame)
    # ))

    return ld