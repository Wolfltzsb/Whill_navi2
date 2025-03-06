from whill_navi2.modules.ros2_launch_utils import (
    get_file_path, get_param_dict,
    DataPath, LaunchDescription, DeclareLaunchArgument, IncludeLaunchDescription,
    PythonLaunchDescriptionSource, Node, ExecuteProcess, LaunchConfiguration,
    FindExecutable, RegisterEventHandler, OnProcessExit,
    OnExecutionComplete, Shutdown, IfCondition, TimerAction
)

def generate_launch_description():
    
    mkdir_params_yaml_path = get_file_path("whill_navi2", "make_dir_node_params.yaml")
    ekf_navsat_params_yaml_path = get_file_path("whill_navi2", "dual_ekf_navsat_params.yaml")
    sensor_launch_path = get_file_path("whill_navi2", "sensor.launch.py")
    kuaro_whill_launch_path = get_file_path("whill_navi2", "kuaro_whill.launch.py")
    rviz_path = get_file_path("whill_navi2", "data_gather_launch.rviz")
    data_path = DataPath()
    # data_path.backup_data("bag")
    
    topic_regex_list = [
        "gnss", "ublox", "tf", "imu", "odom", "velodyne_points", "lumotive_ros"
        # "/zed2i/zed_node/left_raw/image_raw_color",
        # "/zed2i/zed_node/left_raw/camera_info"
        # "/zed2i/zed_node/depth/depth_registered"
    ]
    topic_regex = ".*(" + "|".join(topic_regex_list) + ").*"
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(name="bag_file_path", default_value=data_path.bag_file_path))
    bag_file_path = LaunchConfiguration("bag_file_path")
    ld.add_action(DeclareLaunchArgument(name="use_adis_imu", default_value="false", choices=["False", "false", "True", "true"]))
    use_adis_imu = LaunchConfiguration("use_adis_imu")
    ld.add_action(DeclareLaunchArgument(name="use_wit_imu", default_value="false", choices=["False", "false", "True", "true"]))
    use_wit_imu = LaunchConfiguration("use_wit_imu")
    ld.add_action(DeclareLaunchArgument(name="use_velodyne", default_value="false", choices=["False", "false", "True", "true"]))
    use_velodyne = LaunchConfiguration("use_velodyne")
    ld.add_action(DeclareLaunchArgument(name="use_ublox", default_value="false", choices=["False", "false", "True", "true"]))
    use_ublox = LaunchConfiguration("use_ublox")
    ld.add_action(DeclareLaunchArgument(name="use_hokuyo", default_value="false", choices=["False", "false", "True", "true"]))
    use_hokuyo = LaunchConfiguration("use_hokuyo")
    ld.add_action(DeclareLaunchArgument(name="use_zed_camera", default_value="false", choices=["False", "false", "True", "true"]))
    use_zed_camera = LaunchConfiguration("use_zed_camera")
    ld.add_action(DeclareLaunchArgument(name="use_m20_lidar", default_value="false", choices=["False", "false", "True", "true"]))
    use_m20_lidar = LaunchConfiguration("use_m20_lidar")
    
    # Include Launch File
    # sensor Launch
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_launch_path),
        launch_arguments=[
            ("use_adis_imu", use_adis_imu),
            ("use_wit_imu", use_wit_imu),
            ("use_velodyne", use_velodyne),
            ("use_ublox", use_ublox),
            ("use_hokuyo", use_hokuyo),
            ("use_zed_camera", use_zed_camera),
            ("use_m20_lidar", use_m20_lidar)
        ]
    )
    # navsat_node = Node(
    #     package="robot_localization",
    #     executable="navsat_transform_node",
    #     name="navsat_transform",
    #     parameters=[ekf_navsat_params_yaml_path],
    #     remappings=[
    #         ("gps/fix", "/gnss/fix"),
    #         ("gps/filtered", "/gnss/filtered"),
    #         ("imu", "/zed2i/zed_node/imu/data"),
    #         ("odometry/gps", "/odometry/gnss"),
    #         ("odometry/filterd", "/zed2i/zed_node/odom") 
    #     ],
    #     condition=IfCondition(use_gnss)
    # )
    # ld.add_action(navsat_node)
    
    # ekf_global_config = get_param_dict(ekf_navsat_params_yaml_path, "ekf_filter_node_global")
    # ekf_global_config["publish_tf"] = send_T_map_odom
    # ekf_global_node = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node_global",
    #     parameters=[ekf_global_config],
    #     remappings=[
    #         ("odometry/filtered", "/odometry/filtered/global")
    #     ],
    #     condition=IfCondition(use_gnss)
    # )
    # ld.add_action(ekf_global_node)
    # ekf_local_node = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node_local",
    #     parameters=[ekf_navsat_params_yaml_path],
    #     remappings=[
    #         ("odometry/filtered", "/odometry/filtered/local")
    #     ]
    # )
    # ld.add_action(ekf_local_node)    
    
    # kuaro_whill Launch
    # Parameter YAML file: config/param/ros2_whill_params.yaml
    # Parameter YAML file: config/param/whill_joy2_params.yaml
    kuaro_whill_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(kuaro_whill_launch_path)
    )
    
    # Node
    # Parameter YAML file: config/param/make_dir_node_params.yaml
    make_dir_node = Node(
        package='whill_navi2',
        executable='make_dir_node',
        parameters=[mkdir_params_yaml_path]
    )
    ld.add_action(make_dir_node)
    
    # Rviz config file: config/rviz2/data_gather_launch.rviz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='data_gather_rviz2_node',
        arguments=['-d', rviz_path]
    )
    
    # Process
    ros2bag_record_process = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'bag', 'record', 
            "--regex", topic_regex,
            "--exclude", "(.*)theora(.*)|(.*)compressed(.*)",
            '-o', bag_file_path
        ]
    )
    
    # Register Event
    when_make_dir_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=make_dir_node,
            on_exit=[
                # kuaro_whill_launch,
                sensor_launch
            ]
        )
    )
    ld.add_action(when_make_dir_complete)

    when_launch_complete = RegisterEventHandler(
        OnExecutionComplete(
            target_action=sensor_launch,
            on_completion=[
                TimerAction(period=1.0, actions=[ros2bag_record_process]),
                rviz2_node
            ]
        )
    )
    ld.add_action(when_launch_complete)
    
    when_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz2_node,
            on_exit=[
                Shutdown()
            ]
        )
    )
    ld.add_action(when_rviz_exit)

    return ld
