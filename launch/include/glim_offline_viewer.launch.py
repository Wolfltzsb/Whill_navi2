from whill_navi2.modules.ros2_launch_utils import (
    get_file_path, 
    DataPath, LaunchDescription, Node, LaunchConfiguration, 
    RegisterEventHandler, OnProcessExit, DeclareLaunchArgument,
    OnExecutionComplete, Shutdown, os, LogInfo, GroupAction,
    TimerAction, GroupAction, IncludeLaunchDescription,
    PythonLaunchDescriptionSource, OnProcessStart, get_package_share_directory,
    ExecuteProcess, FindExecutable, IfCondition, UnlessCondition, OpaqueFunction,
    OnShutdown, EmitEvent
)

def generate_launch_description():
    
    data_path = DataPath()
    glim_dump_path = os.path.join(data_path.pcd_map_dir_path, "tmp_dump")
    # tf2_static_launch_path = get_file_path("whill_navi2", "tf2_static.launch.py")
    # glim_waypoint_path = os.path.join(data_path.waypoint_dir_path, "glim_waypoint.yaml")
    
    ld = LaunchDescription()
    # ld.add_action(DeclareLaunchArgument(name="bag_file_path", default_value=data_path.bag_file_path))
    # bag_file_path = LaunchConfiguration("bag_file_path")
    # ld.add_action(DeclareLaunchArgument(name="bag_rate", default_value="20.0"))
    # bag_rate = LaunchConfiguration("bag_rate")
    # ld.add_action(DeclareLaunchArgument(name="bag_start_offset", default_value="0.0"))
    # bag_start_offset = LaunchConfiguration("bag_start_offset")
    # ld.add_action(DeclareLaunchArgument(name="bag_play_duration", default_value="10.0"))
    # bag_play_duration = LaunchConfiguration("bag_play_duration")
    ld.add_action(DeclareLaunchArgument(name="glim_dump_path", default_value=glim_dump_path))
    glim_dump_path = LaunchConfiguration("glim_dump_path")
    # ld.add_action(DeclareLaunchArgument(name="glim_waypoint_path", default_value=glim_waypoint_path))
    # glim_waypoint_path = LaunchConfiguration("glim_waypoint_path")
    # ld.add_action(DeclareLaunchArgument(name="slice_bag", default_value="False"))
    # slice_bag = LaunchConfiguration("slice_bag")
    # ld.add_action(DeclareLaunchArgument("namespace", default_value="glim_slice"))
    # namespace = LaunchConfiguration("namespace")
    
    glim_viewer_node = Node(
        package="glim_ros",
        executable="offline_viewer",
        name="offline_viewer",
        output="screen"
    )
    ld.add_action(glim_viewer_node)
    ply2pcd_node = Node(
        package="whill_navi2",
        executable="ply2pcd_node",
        name="ply2pcd",
        parameters=[{
            "ply_dir_path" : os.getcwd(),
            "pcd_dir_path" : data_path.pcd_map_dir_path,
            "auto_clean_tmp" : "False"
        }]
    )
    ld.add_action(ply2pcd_node)
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=glim_viewer_node,
        on_exit=[Shutdown()]
    )))

    # glim_rosnode = Node(
    #     package="glim_ros",
    #     executable="glim_rosnode",
    #     name="glim_rosnode",
    #     # namespace=namespace,
    #     arguments=[
    #         "--ros-args", "-p", ("dump_path:=", glim_dump_path),
    #         "-p", "auto_quit:=true", "-p", "config_path:={}".format(os.path.join(data_path.params_base_dir_path, "glim_config"))
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
    # rosnode_waypoint_pkg = Node(
    #     package="waypoint_pkg",
    #     executable="waypoint_recorder",
    #     name=(namespace, "_waypoint_recorder"),
    #     parameters=[{
    #         "waypoint_file_path": glim_waypoint_path,
    #         # "source_frame": "map",
    #         # "target_frame": "base_link",
    #         "topic_name": (namespace, "/glim_rosnode/odom"),
    #         "delta_distance": 3.0,
    #         "delta_yaw": 18.0,
    #         "delta_chrod": 2.0,
    #         "with_rviz": False,
    #         "from_topic": True,
    #         "from_gnss": False
    #     }]
    # )
    # play_bag = ExecuteProcess(cmd=[
    #     FindExecutable(name="ros2"), "bag", "play",
    #     bag_file_path,
    #     "--rate", bag_rate, "--start-offset", bag_start_offset,
    #     "--remap", ("/velodyne_points:=", namespace, "/velodyne_points"), ("/adis/imu/data:=", namespace, "/adis/imu/data"),
    #     ("/zed2i/zed_node/left_raw/image_raw_color:=", namespace, "/zed2i/zed_node/left_raw/image_raw_color")
    # ])
    # glim_viewer_node = Node(
    #     package="glim_ros",
    #     executable="offline_viewer",
    #     name="offline_viewer",
    #     namespace=namespace,
    #     arguments=[
    #         glim_dump_path
    #     ],
    #     output="screen"
    # )
    # ply2pcd_node = Node(
    #     package="whill_navi2",
    #     executable="ply2pcd_node",
    #     namespace=namespace,
    #     name="ply2pcd",
    #     parameters=[{
    #         "ply_dir_path" : os.getcwd(),
    #         "pcd_dir_path" : data_path.pcd_map_dir_path,
    #         "auto_clean_tmp" : "False"
    #     }]
    # )
    # sliced_bag_group = GroupAction(
    #     actions=[
    #         glim_rosnode,
    #         RegisterEventHandler(OnProcessStart(
    #             target_action=glim_rosnode,
    #             on_start=[TimerAction(period=0.2, actions=[play_bag, rosnode_waypoint_pkg])]
    #         )),
    #         RegisterEventHandler(OnProcessStart(
    #             target_action=play_bag,
    #             on_start=[TimerAction(period=bag_play_duration, actions=[
    #                 Shutdown()
    #             ])]
    #         )),
            # RegisterEventHandler(OnShutdown(
            #     on_shutdown=[
            #         glim_viewer_node,
            #         ply2pcd_node,
            #         # TimerAction(period=60.0, actions=[LogInfo(msg="Shutdown")])
            #     ]
            # )),
            # RegisterEventHandler(OnProcessExit(
            #     target_action=glim_viewer_node,
            #     on_exit=[TimerAction(period=1.0, actions=[Shutdown()])]
            # ))
            # RegisterEventHandler(OnProcessExit(
            #     target_action=glim_rosnode,
            #     on_exit=[TimerAction(period=0.5, actions=[glim_viewer_node])]
            # )),
            # RegisterEventHandler(OnProcessExit(
            #     target_action=glim_viewer_node,
            #     on_exit=[ply2pcd_node]
            # )),
            # RegisterEventHandler(OnProcessExit(
            #     target_action=ply2pcd_node,
            #     on_exit=[TimerAction(period=0.2, actions=[Shutdown()])]
            # ))
    #     ],
    #     condition=IfCondition(slice_bag)
    # )
    # ld.add_action(sliced_bag_group)
    
    # glim_rosbag_node = Node(
    #     package="glim_ros",
    #     executable="glim_rosbag",
    #     name="glim_rosbag",
    #     arguments=[
    #         bag_file_path,
    #         "--ros-args", "-p", ("dump_path:=", glim_dump_path),
    #         "-p", "auto_quit:=true", "-p", "config_path:={}".format(os.path.join(data_path.params_base_dir_path, "glim_config"))
    #     ],
    #     # parameters=[{
    #     #     "dump_path" : glim_dump_path,
    #     #     "auto_quit" : "true"
    #     # }],
    #     output="screen"
    # )
    # rosbag_waypoint_pkg = Node(
    #     package="waypoint_pkg",
    #     executable="waypoint_recorder",
    #     namespace="glim_rosbag",
    #     parameters=[{
    #         "waypoint_file_path": glim_waypoint_path,
    #         # "source_frame": "map",
    #         # "target_frame": "base_link",
    #         "topic_name": "/glim_rosbag/odom",
    #         "delta_distance": 3.0,
    #         "delta_yaw": 18.0,
    #         "delta_chrod": 2.0,
    #         "with_rviz": False,
    #         "from_topic": True,
    #         "from_gnss": False
    #     }]
    # )

    # full_bag_group = GroupAction(
    #     actions=[
    #         glim_rosbag_node,
    #         RegisterEventHandler(OnProcessStart(
    #             target_action=glim_rosbag_node,
    #             on_start=[TimerAction(period=0.05, actions=[rosbag_waypoint_pkg])]
    #         )),
    #         RegisterEventHandler(OnProcessExit(
    #             target_action=glim_rosbag_node,
    #             on_exit=[glim_viewer_node]
    #         )),
    #         RegisterEventHandler(OnProcessExit(
    #             target_action=glim_viewer_node,
    #             on_exit=[ply2pcd_node]
    #         )),
    #         RegisterEventHandler(OnProcessStart(
    #             target_action=ply2pcd_node,
    #             on_start=[TimerAction(period=5.0, actions=[Shutdown()])]
    #         ))
    #     ],
    #     condition=UnlessCondition(slice_bag)
    # )
    # ld.add_action(full_bag_group)
    
    # tf2_static_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(tf2_static_launch_path),
    #     launch_arguments=[
    #         ("use_velodyne_frame", "true"),
    #         ("use_adis_imu_frame", "true")
    #     ]
    # )
    # ld.add_action(tf2_static_launch)
                
    return ld
        
    
    