from whill_navi2.modules.ros2_launch_utils import (
    IncludeLaunchDescription, LaunchDescription, PythonLaunchDescriptionSource,
    Node, RegisterEventHandler, OnProcessStart, OnProcessExit, 
    DataPath, get_file_path, GroupAction, TimerAction,
    ExecuteProcess, FindExecutable, Shutdown, OnExecutionComplete,
    os, DeclareLaunchArgument, LaunchConfiguration,
    get_param_dict
)

def generate_launch_description():
    ld = LaunchDescription()
    data_path = DataPath()
    ld.add_action(DeclareLaunchArgument(name="globalmap_pcd", default_value=os.path.join(data_path.pcd_map_dir_path, "glim.pcd")))
    globalmap_pcd = LaunchConfiguration("globalmap_pcd")
    ld.add_action(DeclareLaunchArgument(name="bag_file_path", default_value=data_path.bag_file_path))
    bag_file_path = LaunchConfiguration("bag_file_path")
    
    ld.add_action(DeclareLaunchArgument(name="init_pos_x", default_value="0.0"))
    init_pos_x = LaunchConfiguration("init_pos_x")
    ld.add_action(DeclareLaunchArgument(name="init_pos_y", default_value="0.0"))
    init_pos_y = LaunchConfiguration("init_pos_y")
    ld.add_action(DeclareLaunchArgument(name="init_pos_z", default_value="0.0"))
    init_pos_z = LaunchConfiguration("init_pos_z")
    ld.add_action(DeclareLaunchArgument(name="init_ori_x", default_value="0.0"))
    init_ori_x = LaunchConfiguration("init_ori_x")
    ld.add_action(DeclareLaunchArgument(name="init_ori_y", default_value="0.0"))
    init_ori_y = LaunchConfiguration("init_ori_y")
    ld.add_action(DeclareLaunchArgument(name="init_ori_z", default_value="0.0"))
    init_ori_z = LaunchConfiguration("init_ori_z")
    ld.add_action(DeclareLaunchArgument(name="init_ori_w", default_value="1.0"))
    init_ori_w = LaunchConfiguration("init_ori_w")
    
    ld.add_action(DeclareLaunchArgument(name="bag_play_duration", default_value="0.0"))
    bag_play_duration = LaunchConfiguration("bag_play_duration")
    ld.add_action(DeclareLaunchArgument(name="bag_play_start_time", default_value="0.0"))
    bag_play_start_time = LaunchConfiguration("bag_play_start_time")
    ld.add_action(DeclareLaunchArgument(name="namespace", default_value=""))
    namespace = LaunchConfiguration("namespace")
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        arguments=["-d", get_file_path("whill_navi2", "hdl_localization_waypoint.rviz")]
    )
    ld.add_action(rviz_node)
    
    hdl_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_file_path("whill_navi2", "hdl_localization.launch.py")),
        launch_arguments=[
            ("use_sim_time", "True"),
            ("globalmap_pcd", globalmap_pcd),
            ("imu_topic", (namespace, "/adis/imu/data")),
            ("point_cloud_topic", (namespace, "/velodyne_points")),
            ("init_pos_x", init_pos_x),
            ("init_pos_y", init_pos_y),
            ("init_pos_z", init_pos_z),
            ("init_ori_x", init_ori_x),
            ("init_ori_y", init_ori_y),
            ("init_ori_z", init_ori_z),
            ("init_ori_w", init_ori_w),
            ("globalmap_resolution", "0.1"),
            ("scaning_resolution", "0.1"),
            ("send_tf_transforms", "True"),
            ("namespace", namespace)
        ]
    )
    # glim_rosnode = Node(
    #     package="glim_ros",
    #     executable="glim_rosnode",
    #     name="glim_rosnode",
    #     namespace=namespace,
    #     arguments=[
    #         "--ros-args", 
    #         # "--param", ("dump_path:=", glim_dump_path),
    #         "--param", "auto_quit:=true", "--param", "config_path:={}".format(os.path.join(data_path.params_base_dir_path, "glim_for_localization_config"))
    #     ],
    #     # parameters=[{
    #     #     "dump_path" : glim_dump_path,
    #     #     "auto_quit" : "true"
    #     # }],
    #     remappings=[
    #         ("/velodyne_points", (namespace, "/velodyne_points")),
    #         ("/adis/imu/data", (namespace, "/adis/imu/data")),
    #         ("/zed2i/zed_node/left_raw/image_raw_color", (namespace, "/zed2i/zed_node/left_raw/image_raw_color")),
    #         ("/glim_rosnode/odom", (namespace, "/glim_rosnode/odom"))
    #     ],
    #     output="screen"
    # )
    # ld.add_action(glim_rosnode)
    waypont_recorder = Node(
        package="waypoint_pkg",
        executable="waypoint_recorder",
        namespace=namespace,
        parameters=[{
            "waypoint_file_path": (data_path.waypoint_dir_path, "/waypoint", namespace, ".yaml"),
            "source_frame": "map",
            "target_frame": "base_link",
            "delta_distance": 5.0,
            "delta_yaw": 10.0,
            "delta_chrod": 2.0,
            "with_rviz": True
        }]
    )
    pickup_vio_waypoint = Node(
        package="waypoint_pkg",
        executable="waypoint_recorder",
        namespace=(namespace, "/zed2i_vio"),
        parameters=[{
            "waypoint_file_path": (data_path.waypoint_dir_path, "/zed2i_vio_odometry", namespace, ".yaml"),
            "delta_distance": 1.0,
            "delta_yaw": 3.0,
            "delta_chrod": 0.5,
            "with_rviz": False,
            "from_topic": True,
            "topic_name": (namespace, "/zed2i/zed_node/odom"),
            "from_gnss": False
        }]
    )
    # pickup_gnss_waypoint = Node(
    #     package="waypoint_pkg",
    #     executable="waypoint_recorder",
    #     namespace="gnss_odom",
    #     parameters=[{
    #         "waypoint_file_path": (data_path.waypoint_dir_path, "/gnss_odometry", namespace, ".yaml"),
    #         "delta_distance": 1.0,
    #         "delta_yaw": 3.0,
    #         "delta_chrod": 0.5,
    #         "with_rviz": False,
    #         "from_topic": True,
    #         "topic_name": (namespace, "/odometry/gnss"),
    #         "from_gnss": False
    #     }]
    # )
    pickup_wheel_odometry = Node(
        package="waypoint_pkg",
        executable="waypoint_recorder",
        namespace="wheel_odom",
        parameters=[{
            "waypoint_file_path": (data_path.waypoint_dir_path, "/whill_odometry", namespace, ".yaml"),
            "delta_distance": 1.0,
            "delta_yaw": 3.0,
            "delta_chrod": 0.5,
            "with_rviz": False,
            "from_topic": True,
            "topic_name": (namespace, "/whill/odometry"),
            "from_gnss": False
        }]
    )
    # pickup_local_ekf_odometry = Node(
    #     package="waypoint_pkg",
    #     executable="waypoint_recorder",
    #     namespace="local_ekf_odometry",
    #     parameters=[{
    #         "waypoint_file_path": (data_path.waypoint_dir_path, "/local_ekf_waypoint", namespace, ".yaml"),
    #         "delta_distance": 1.0,
    #         "delta_yaw": 3.0,
    #         "delta_chrod": 0.5,
    #         "with_rviz": False,
    #         "from_topic": True,
    #         "topic_name": (namespace, "/odometry/filtered/local"),
    #         "from_gnss": False
    #     }]
    # )
    pickup_lonlat = Node(
        package="waypoint_pkg",
        executable="waypoint_recorder",
        namespace="lonlat",
        parameters=[{
            "waypoint_file_path": (data_path.waypoint_dir_path, "/GeoLocalization", namespace, ".yaml"),
            "with_rviz": False,
            "from_topic": True,
            "topic_name": (namespace, "/gnss/fix"),
            "from_gnss": True
        }]
    )
    pickup_hdl_odometry = Node(
        package="waypoint_pkg",
        executable="waypoint_recorder",
        namespace="hdl_localization",
        parameters=[{
            "waypoint_file_path": (data_path.waypoint_dir_path, "/hdl_localization_waypoint", namespace, ".yaml"),
            "with_rviz": False,
            "from_topic": True,
            "topic_name": (namespace, "/hdl_localization/odom"),
            "from_gnss": False
        }]
    )
    pickup_waypoint_group = GroupAction(actions=[
        pickup_vio_waypoint,
        # pickup_gnss_waypoint,
        pickup_lonlat,
        waypont_recorder,
        pickup_wheel_odometry,
        pickup_hdl_odometry,
        # pickup_local_ekf_odometry
    ])
    
    # ekf_navsat_params_yaml_path = get_file_path("whill_navi2", "dual_ekf_navsat_params.yaml")
    # local_ekf_params_dict = get_param_dict(ekf_navsat_params_yaml_path, "ekf_filter_node_local")
    # global_ekf_params_dict = get_param_dict(ekf_navsat_params_yaml_path, "ekf_filter_node_global")
    # navsat_transform_params_dict = get_param_dict(ekf_navsat_params_yaml_path, "navsat_transform")
    # # dual_navsat
    # local_ekf_params_dict["odom0"] = (namespace, local_ekf_params_dict["odom0"])
    # local_ekf_params_dict["odom1"] = (namespace, local_ekf_params_dict["odom1"])
    # local_ekf_params_dict["odom2"] = (namespace, local_ekf_params_dict["odom2"])
    # tmp_list = []
    # for ele in local_ekf_params_dict["initial_estimate_covariance"]:
    #     tmp_list.append(float(ele))
    
    # local_ekf_params_dict["initial_estimate_covariance"] = tmp_list
    
    # tmp_list.clear()
    # for ele in global_ekf_params_dict["initial_estimate_covariance"]:
    #     tmp_list.append(float(ele))
    # global_ekf_params_dict["odom0"] = (namespace, global_ekf_params_dict["odom0"])
    # global_ekf_params_dict["imu0"] = (namespace, global_ekf_params_dict["imu0"])
    # global_ekf_params_dict["initial_estimate_covariance"] = tmp_list
    # ld.add_action(TimerAction(
    #     period=0.1,
    #     actions=[
    #         Node(
    #             package='robot_localization',
    #             executable='ekf_node',
    #             name='ekf_filter_node_local',
    #             namespace=namespace,
    #             parameters=[
    #                 local_ekf_params_dict
    #             ],
    #             remappings=[
    #                 ((namespace, "/odometry/filtered"), (namespace, "/odometry/filtered/local"))
    #             ]
    #         ),
    #         Node(
    #             package='robot_localization',
    #             executable='ekf_node',
    #             name='ekf_filter_node_global',
    #             namespace=namespace,
    #             parameters=[
    #                 global_ekf_params_dict
    #             ],
    #             remappings=[
    #                 ((namespace, "odometry/filtered"), (namespace, "/odometry/filtered/global"))
    #             ]
    #         ),
    #         Node(
    #             package='robot_localization',
    #             executable='navsat_transform_node',
    #             namespace=namespace,
    #             parameters=[
    #                 navsat_transform_params_dict
    #             ],
    #             remappings=[
    #                 ((namespace, "gps/fix"), (namespace, "/gnss/fix")),
    #                 ((namespace, "gps/filtered"), (namespace, "/gnss/filtered")),
    #                 ((namespace, "imu"), (namespace, "/zed2i/zed_node/imu/data_enu")),
    #                 ((namespace, "odometry/gps"), (namespace, "/odometry/gnss")),
    #                 ((namespace, "odometry/filtered"), (namespace, "/odometry/filtered/global"))
    #             ]
    #         )
    #     ]
    # ))
    
    when_rviz_starting = RegisterEventHandler(
        OnProcessStart(
            target_action=rviz_node,
            on_start=[TimerAction(period=1.0, actions=[hdl_localization_launch, pickup_waypoint_group])]
        )
    )
    ld.add_action(when_rviz_starting)
    
    play_bag = ExecuteProcess(cmd=[
        FindExecutable(name="ros2"), "bag", "play",
        "--rate", "2.0", bag_file_path,
        "--start-offset", bag_play_start_time,
        "--remap", ("/velodyne_points:=", namespace, "/velodyne_points"),
        ("/adis/imu/data:=", namespace, "/adis/imu/data"),
        ("/zed2i/zed_node/left_raw/image_raw_color:=", namespace, "/zed2i/zed_node/left_raw/image_raw_color"),
        ("/zed2i/zed_node/odom:=", namespace, "/zed2i/zed_node/odom"),
        ("/zed2i/zed_node/imu/data_enu:=", namespace, "/zed2i/zed_node/imu/data_enu"),        
        ("/gnss/fix:=", namespace, "/gnss/fix"),        
        ("/whill/odometry:=", namespace, "/whill/odometry")
    ])

    when_hdl_starting = RegisterEventHandler(
        OnExecutionComplete(
            target_action=hdl_localization_launch,
            on_completion=[TimerAction(
                    period=8.0, 
                    actions=[play_bag],
            )]
        )
    )
    ld.add_action(when_hdl_starting)
    
    play_bag_timer = TimerAction(period=bag_play_duration, actions=[Shutdown()])
    when_bag_start = RegisterEventHandler(
        OnProcessStart(
            target_action=play_bag,
            on_start=[play_bag_timer]
        )
    )
    ld.add_action(when_bag_start)
    
    # when_play_bag_exiting = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=play_bag,
    #         on_exit=[Shutdown(reason="Data Bag playing is over... Shutdown the launch")]
    #     )
    # )
    # ld.add_action(when_play_bag_exiting)
    
    return ld