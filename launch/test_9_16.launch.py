from whill_navi2.modules.ros2_launch_utils import (
    get_file_path,
    DataPath, LaunchDescription, IncludeLaunchDescription,
    PythonLaunchDescriptionSource, Node, ExecuteProcess,
    RegisterEventHandler, OnProcessStart, TimerAction, FindExecutable,
    OnProcessExit, Shutdown, os, DeclareLaunchArgument, LaunchConfiguration
)

def generate_launch_description():
    
    ld = LaunchDescription()

    # sensor_launch_path = get_file_path("whill_navi2", "sensor.launch.py")
    # kuaro_launch_path = get_file_path("whill_navi2", "kuaro_whill.launch.py")
    tf2_static_launch_path = get_file_path("whill_navi2", "tf2_static.launch.py")
    # nav2_no_map_params_path = get_file_path("whill_navi2", "nav2_no_map_params.yaml")
    navsat_ekf_params_path = get_file_path("whill_navi2", "dual_ekf_navsat_params.yaml")
    # navigation_rviz_path = get_file_path("whill_navi2", "navigation.rviz")
    # mapviz_path = get_file_path("whill_navi2", "waypoint_generator_launch.mvc")
    glim_dump_path = os.path.join(DataPath().pcd_map_dir_path, "test_tmp_dump")
    ld.add_action(DeclareLaunchArgument(name="glim_dump_path", default_value=glim_dump_path))
    glim_dump_path = LaunchConfiguration("glim_dump_path")
    ld.add_action(DeclareLaunchArgument(name="globalmap_pcd", default_value=os.path.join(DataPath().pcd_map_dir_path, "part1.pcd")))
    globalmap_pcd = LaunchConfiguration("globalmap_pcd")
    ld.add_action(DeclareLaunchArgument(name="data_bag_path", default_value=DataPath().bag_file_path))
    data_bag_path = LaunchConfiguration("data_bag_path")
    ld.add_action(DeclareLaunchArgument(name="bag_rate", default_value="5.0"))
    bag_rate = LaunchConfiguration("bag_rate")

##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    # launch glim rosnode
    glim_rosnode = Node(
        package="glim_ros",
        executable="glim_rosnode",
        name="glim_rosnode",
        # namespace=namespace,
        arguments=[
            "--ros-args", "-p", ("dump_path:=", glim_dump_path),
            "-p", "auto_quit:=true", "-p", "config_path:={}".format(os.path.join(DataPath().params_base_dir_path, "glim_for_localization_config"))
        ],
        # parameters=[{
        #     "dump_path" : glim_dump_path,
        #     "auto_quit" : "true"
        # }],
        output="screen"
    )
    ld.add_action(glim_rosnode)
    # launch hdl localization
    # hdl_localization_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(get_file_path("whill_navi2", "hdl_localization.launch.py")),
    #     launch_arguments=[
    #         ("use_sim_time", "True"),
    #         ("globalmap_pcd", globalmap_pcd),
    #         ("imu_topic", "/adis/imu/data"),
    #         ("globalmap_resolution", "0.35"),
    #         ("scaning_resolution", "0.35"),
    #         ("send_tf_transforms", "False")
    #     ]
    # )
    # ld.add_action(hdl_localization_launch)
    # dual_navsat
    ld.add_action(TimerAction(
        period=0.1,
        actions=[
            # Node(
            #     package='robot_localization',
            #     # executable='ekf_node',
            #     executable='ukf_node',
            #     name='ekf_filter_node_local',
            #     parameters=[
            #         navsat_ekf_params_path
            #     ],
            #     remappings=[
            #         ("odometry/filtered", "/odometry/filtered/local")
            #     ]
            # ),
            # Node(
            #     package='robot_localization',
            #     # executable='ekf_node',
            #     executable='ukf_node',
            #     name='ekf_filter_node_global',
            #     parameters=[
            #         navsat_ekf_params_path,
            #         {
            #             "kappa": - 3.0
            #         }
            #     ],
            #     remappings=[
            #         ("odometry/filtered", "/odometry/filtered/global")
            #     ]
            # ),
            # Node(
            #     package='robot_localization',
            #     executable='navsat_transform_node',
            #     parameters=[
            #         navsat_ekf_params_path
            #     ],
            #     remappings=[
            #         ("gps/fix", "/gnss/fix"),
            #         ("gps/filtered", "/gnss/filtered"),
            #         ("imu", "/zed2i/zed_node/imu/data_enu"),
            #         ("odometry/gps", "/odometry/gnss"),
            #         # ("odometry/filtered", "/odometry/filtered/global"),
            #         ("odometry/filtered", "/odometry/filtered/global")
            #     ]
            # )
        ]
    ))
    play_data_bag = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "bag", "play", data_bag_path,
            "--rate", bag_rate
        ]
    )
    ld.add_action(play_data_bag)
    record_data_bag = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "bag", "record", 
            "--regex", ".*(odom|nmea).*",
            "--output", os.path.join(DataPath().bag_dir_path, "..", "Backup", os.path.basename(DataPath().bag_file_path) + "_for_evo_eval")
        ]
    )
    ld.add_action(record_data_bag)

    tf2_static_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            tf2_static_launch_path
        ),
        launch_arguments=[
            ("use_velodyne_frame", "true"),
            ("use_adis_imu_frame", "true"),
            ("use_zed_camera_frame", "true")
        ]
    )
    ld.add_action(tf2_static_launch)
    
    # ld.add_action(Node(
    #     package="waypoint_pkg",
    #     executable="waypoint_recorder",
    #     name="test_ekf_filter_waypoint_recorder",
    #     parameters=[{
    #         "waypoint_file_path": os.path.join(DataPath().waypoint_dir_path, "test_ekf_waypoint.yaml"),
    #         "source_frame": "map",
    #         "target_frame": "base_link",
    #         "delta_distance": 5.0,
    #         "delta_yaw": 10.0,
    #         "delta_chrod": 2.0,
    #         "with_rviz": False,
    #         "from_topic": True,
    #         "from_gnss": False,
    #         "topic_name": "/odometry/filtered/global"
    #     }]
    # ))
    
    # ld.add_action(Node(
    #     package="waypoint_pkg",
    #     executable="waypoint_recorder",
    #     name="test_ekf_local_waypoint_recorder",
    #     parameters=[{
    #         "waypoint_file_path": os.path.join(DataPath().waypoint_dir_path, "test_ekf_local_waypoint.yaml"),
    #         "source_frame": "odom",
    #         "target_frame": "base_link",
    #         "delta_distance": 5.0,
    #         "delta_yaw": 10.0,
    #         "delta_chrod": 2.0,
    #         "with_rviz": False,
    #         "from_topic": True,
    #         "from_gnss": False,
    #         "topic_name": "/odometry/filtered/local"
    #     }]
    # ))    
    
    # ld.add_action(Node(
    #     package="waypoint_pkg",
    #     executable="waypoint_recorder",
    #     name="test_ekf_local_transformed_waypoint_recorder",
    #     parameters=[{
    #         "waypoint_file_path": os.path.join(DataPath().waypoint_dir_path, "test_ekf_local_transformed_waypoint.yaml"),
    #         "source_frame": "odom",
    #         "target_frame": "base_link",
    #         "delta_distance": 5.0,
    #         "delta_yaw": 10.0,
    #         "delta_chrod": 2.0,
    #         "with_rviz": False,
    #         "from_topic": True,
    #         "from_gnss": False,
    #         "topic_name": "/odometry/filtered/local/transformed"
    #     }]
    # ))
    
    ld.add_action(Node(
        package="waypoint_pkg",
        executable="waypoint_recorder",
        name="test_origin_glim_odometry",
        parameters=[{
            "waypoint_file_path": os.path.join(DataPath().waypoint_dir_path, "test_origin_glim_odometry.yaml"),
            "source_frame": "odom",
            "target_frame": "base_link",
            "delta_distance": 5.0,
            "delta_yaw": 10.0,
            "delta_chrod": 2.0,
            "with_rviz": False,
            "from_topic": True,
            "from_gnss": False,
            "topic_name": "/glim_rosnode/odom"
        }]
    ))
    
    # ld.add_action(Node(
    #     package="waypoint_pkg",
    #     executable="waypoint_recorder",
    #     namespace="test_navsat_transforms",
    #     parameters=[{
    #         "waypoint_file_path": os.path.join(DataPath().waypoint_dir_path, "test_navsat_waypoint.yaml"),
    #         "source_frame": "map",
    #         "target_frame": "base_link",
    #         "delta_distance": 5.0,
    #         "delta_yaw": 10.0,
    #         "delta_chrod": 2.0,
    #         "with_rviz": False,
    #         "from_topic": True,
    #         "from_gnss": True,
    #         "topic_name": "/gnss/filtered",
    #     }]
    # ))
    
    # ld.add_action(Node(
    #     package="waypoint_pkg",
    #     executable="waypoint_recorder",
    #     namespace="test_gnss_odometry",
    #     parameters=[{
    #         "waypoint_file_path": os.path.join(DataPath().waypoint_dir_path, "test_gnss_odometry.yaml"),
    #         "source_frame": "map",
    #         "target_frame": "base_link",
    #         "delta_distance": 5.0,
    #         "delta_yaw": 10.0,
    #         "delta_chrod": 2.0,
    #         "with_rviz": False,
    #         "from_topic": True,
    #         "from_gnss": False,
    #         "topic_name": "/odometry/gnss",
    #     }]
    # ))
    
    ld.add_action(Node(
        package="waypoint_pkg",
        executable="waypoint_recorder",
        namespace="test_raw_gnss",
        parameters=[{
            "waypoint_file_path": os.path.join(DataPath().waypoint_dir_path, "test_raw_gnss_waypoint.yaml"),
            "source_frame": "map",
            "target_frame": "base_link",
            "delta_distance": 5.0,
            "delta_yaw": 10.0,
            "delta_chrod": 2.0,
            "with_rviz": False,
            "from_topic": True,
            "from_gnss": True,
            "topic_name": "/gnss/fix",
        }]
    ))
    ld.add_action(Node(
        package="waypoint_pkg",
        executable="waypoint_recorder",
        namespace="test_raw_gnss",
        parameters=[{
            "waypoint_file_path": os.path.join(DataPath().waypoint_dir_path, "lio_gnss_waypoint.yaml"),
            "source_frame": "topomap",
            "target_frame": "base_link",
            "delta_distance": 1.0,
            "delta_yaw": 5.0,
            "delta_chrod": 0.8,
            "with_rviz": False,
            "from_topic": False,
            "from_gnss": False,
            # "topic_name": "/gnss/fix",
        }]
    ))
    
    
    # Publish Imu Data with enu frame
    # imu_filter_node = Node(
    #     package="imu_filter_madgwick",
    #     executable="imu_filter_madgwick_node",
    #     parameters=[{
    #         "publish_tf" : False,
    #         "use_mag" : True,
    #         # "mag_bias_x" : - 0.6800175327262878,
    #         # "mag_bias_y" : 0.5951328873634338,
    #         # "mag_bias_z" : 0.40913358330726624,
    #         "world_frame" : "enu"
    #     }],
    #     remappings=[
    #         ("imu/data_raw", "{}/{}/imu/data".format("zed2i", "zed_node")),
    #         ("imu/data", "{}/{}/imu/data_enu".format("zed2i", "zed_node")),
    #         ("imu/mag", "{}/{}/imu/mag".format("zed2i", "zed_node"))
    #     ]
    # )
    # ld.add_action(imu_filter_node)

    
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=play_data_bag,
        on_exit=[Shutdown()]
    )))

    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", os.path.join(os.environ["HOME"], "Documents", "test_hdl_localization.rviz")]
    # )
    # navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(get_file_path("whill_navi2", "navigation.launch.py")),
    #     launch_arguments=[["params_file", get_file_path("whill_navi2", "nav2_params.yaml")]]
    # )
    # ld.add_action(rviz)
    # ld.add_action(navigation)
    
    # ekf_node = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node_local",
    #     parameters=[navsat_ekf_params_path],
    #     remappings=[("odometry/filtered", "odometry/filtered/local")]
    # )
    # ld.add_action(ekf_node)
    
    # play_bag = ExecuteProcess(
    #     cmd=[
    #         FindExecutable(name="ros2"),
    #         "bag", "play", DataPath().bag_file_path
    #     ]
    # )
    # ld.add_action(TimerAction(period=0.5, actions=[play_bag]))
    
    # when_bagplay_over = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=play_bag,
    #         on_exit=[
    #             TimerAction(period=0.25, actions=[Shutdown(reason="Bag is over!")])
    #         ]
    #     )
    # )
    # ld.add_action(when_bagplay_over)
    
    # ld.add_action(Node(
    #     package="swri_transform_util",
    #     executable="initialize_origin.py",
    #     name="initialize_origin_node",
    #     remappings=[
    #         ("fix", "/gnss/fix")
    #     ],
    #     parameters=[{}]
    # ))
    # ld.add_action(Node(
    #     package="mapviz",
    #     executable="mapviz",
    #     name="lio_sam_mapviz",
    # ))
    # ld.add_action(Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="map_to_origin",
    #     arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "topomap", "--child-frame-id", "origin"]
    # ))


    
    return ld
    
