from whill_navi2.modules.ros2_launch_utils import (
    get_file_path,
    LaunchDescription, DeclareLaunchArgument,
    LaunchConfiguration, GroupAction, IncludeLaunchDescription,
    PythonLaunchDescriptionSource, IfCondition
)

def generate_launch_description():
    
    # Launch file path
    adis_imu_launch_path = get_file_path("whill_navi2", 'adis16465.launch.py')
    wit_imu_launch_path = get_file_path("whill_navi2", 'wt901.launch.py')
    zed_f9p_launch_path = get_file_path("whill_navi2", 'zed_f9p.launch.py')
    urg_node_launch_path = get_file_path("whill_navi2", 'urg_node2.launch.py')
    web_camera_launch_path = get_file_path("whill_navi2", 'web_camera.launch.py')
    velodyne_launch_path = get_file_path("whill_navi2", 'velodyne-all-nodes-VLP16-launch.py')
    zed_camera_launch_path = get_file_path("whill_navi2", "zed_camera.launch.py")
    m20_launch_path = get_file_path("whill_navi2", "m20_launcher.launch.py")
    tf2_static_launch_path = get_file_path("whill_navi2", "tf2_static.launch.py")

##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################

    ld = LaunchDescription()
        
    # Declare Launch Arguments
    ld.add_action(DeclareLaunchArgument("use_adis_imu", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_wit_imu", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_velodyne", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_ublox", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_hokuyo", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_web_camera", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_realsense_camera", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_zed_camera", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_m20_lidar", default_value="false"))
    use_adis_imu = LaunchConfiguration("use_adis_imu")
    use_wit_imu = LaunchConfiguration("use_wit_imu")
    use_velodyne = LaunchConfiguration("use_velodyne")
    use_ublox = LaunchConfiguration("use_ublox")
    use_hokuyo = LaunchConfiguration("use_hokuyo")
    use_web_camera = LaunchConfiguration("use_web_camera")
    # use_realsense_camera = LaunchConfiguration("use_realsense_camera")
    use_zed_camera = LaunchConfiguration("use_zed_camera")
    use_m20_lidar = LaunchConfiguration("use_m20_lidar")
      
    # Include Launch file
    # ADIS IMU Launch
    adis_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(adis_imu_launch_path),
        condition=IfCondition(use_adis_imu)
    )
    ld.add_action(adis_imu_launch)
    # WIT IMU Launch
    # Parameter YAML file: config/params/wt901_params.yaml
    wit_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(wit_imu_launch_path),
        condition=IfCondition(use_wit_imu)
    )
    ld.add_action(wit_imu_launch)
    # Ublox GNSS Launch
    # Launch Argument YAML file: config/launch_arg/ublox_gnss_launch_arg.yaml
    ublox_gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_f9p_launch_path),
        condition=IfCondition(use_ublox)        
    )
    ld.add_action(ublox_gnss_launch)
    # Hokuyo LRF Launch
    # Launch Argument YAML file: config/launch_arg/hokuyo_lrf_launch_arg.yaml
    # Parameter YAML file: config/params/hokuyo_urg_node_params.yaml
    hokuyo_lrf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urg_node_launch_path),
        launch_arguments=[
            ("auto_start", 'true'),
            ("node_name", 'urg_node2'),
            ("with_rviz", 'false')
        ],
        condition=IfCondition(use_hokuyo)        
    )
    ld.add_action(hokuyo_lrf_launch)
    # Web Camera Launch
    # Parameter YAML file: config/params/web_camera_node_params.yaml
    web_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(web_camera_launch_path),
        condition=IfCondition(use_web_camera)        
    )
    ld.add_action(web_camera_launch)
    # Velodyne Launch
    # Parameter YAML file: config/params/VLP16-velodyne_driver_node-params.yaml
    # Parameter YAML file: config/params/VLP16-velodyne_transform_node-params.yaml
    # Parameter YAML file: config/params/VLP16db.yaml
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velodyne_launch_path),
        launch_arguments=[
            ["with_rviz", "false"],
        ],
        condition=IfCondition(use_velodyne)        
    )
    ld.add_action(velodyne_launch)
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_camera_launch_path),
        launch_arguments=[
            ["publish_tf", "false"],
            ["publish_map_tf", "false"],
            ["camera_model", "zed2i"],
            ["camera_name", "zed2i"]
        ],
        condition=IfCondition(use_zed_camera)
    )
    ld.add_action(zed_camera_launch)
    
    # lumotive m20 lidar
    # Parameter file: config/params/m20_lidar_node_params.yaml
    m20_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(m20_launch_path),
        condition=IfCondition(use_m20_lidar)
    )
    ld.add_action(m20_lidar_launch)
    
    # tf2_static Launch
    tf2_static_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf2_static_launch_path),
        launch_arguments=[
            ("use_hokuyo_frame", use_hokuyo),
            ("use_velodyne_frame", use_velodyne),
            ("use_adis_imu_frame", use_adis_imu),
            ("use_wit_imu_frame", use_wit_imu),
            ("use_ublox_frame", use_ublox),
            ("use_m20_lidar_frame", use_m20_lidar),
            ("use_zed_camera_frame", use_zed_camera)
        ]
    )
    ld.add_action(tf2_static_launch)
    return ld
