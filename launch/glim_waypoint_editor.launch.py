from whill_navi2.modules.ros2_launch_utils import (
    Node, IncludeLaunchDescription, DataPath, LaunchDescription,
    get_file_path, get_param_dict, os, OnProcessStart,
    RegisterEventHandler, TimerAction, OnProcessExit, Shutdown,
    PythonLaunchDescriptionSource, get_package_share_directory,
    ExecuteProcess, FindExecutable, DeclareLaunchArgument, LaunchConfiguration
)

def generate_launch_description():

    ld = LaunchDescription()
    data_path = DataPath()
    rviz2_file_path = get_file_path("whill_navi2", "glim_waypoint_editor.rviz")
    
    ld.add_action(DeclareLaunchArgument(name="globalmap_pcd", default_value=os.path.join(data_path.pcd_map_dir_path, "glim.pcd")))
    globalmap_pcd = LaunchConfiguration("globalmap_pcd")
    ld.add_action(DeclareLaunchArgument(name="waypoint_file_path", default_value=data_path.waypoint_file_path))
    waypoint_file_path = LaunchConfiguration("waypoint_file_path")
    ld.add_action(DeclareLaunchArgument(name="rewaypoint_file_path", default_value=data_path.rewaypoint_file_path))
    rewaypoint_file_path = LaunchConfiguration("rewaypoint_file_path")

    hdl_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_file_path("whill_navi2", "hdl_localization.launch.py")),
        launch_arguments=[
            ["plot_estimation_errors", "false"],
            ["globalmap_pcd", globalmap_pcd]
        ]
    )
    map_saver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("nav2_map_server"),
                "launch", 
                "map_saver_server.launch.py"
            )
        )
    )
    send_T_map_zed2i = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed2i_to_map",
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', "map", 
            '--child-frame-id', "zed2i_left_camera_optical_frame",             
        ]
    )
    ld.add_action(send_T_map_zed2i)
    
    save_map = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"), "service", "call", 
            "/map_saver/save_map", "nav2_msgs/srv/SaveMap", 
            "{{map_topic: {}, map_url: {}, image_format: {}, map_mode: {}, free_thresh: {}, occupied_thresh: {}}}".format(
            "/map", data_path.map_file_path, "pgm", "", 0.0, 0.0)
        ]
    )
    ld.add_action(TimerAction(period=10.0, actions=[save_map]))
    
    waypoint_editor_node = Node(
        package="waypoint_pkg",
        executable="waypoint_editor",
        name="waypoint_editor",
        parameters=[{
            "read_file_name": waypoint_file_path,
            "write_file_name": rewaypoint_file_path
        }]
    )
    
    # play_bag_for_image = ExecuteProcess(
    #     cmd=[
    #         FindExecutable(name="ros2"), "bag", "play", 
    #         data_path.bag_file_path,
    #         "--loop", "--rate", "5.0",
    #         "--topics", "/zed2i/zed_node/left_raw/image_raw_color", "/zed2i/zed_node/left_raw/camera_info"
    #     ]
    # )
    
    # Rviz config file: config/rviz2/data_gather_launch.rviz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='glim_waypoint_editor_rviz2',
        arguments=['-d', rviz2_file_path]
    )
    ld.add_action(rviz2_node)
    
    when_rviz2_starting = RegisterEventHandler(
        OnProcessStart(
            target_action=rviz2_node,
            on_start=[
                TimerAction(
                    period=0.3, 
                    actions=[
                        waypoint_editor_node,
                        hdl_localization_launch
                    ]
                ),
                TimerAction(
                    period=5.0,
                    actions=[
                        map_saver_launch,
                        # play_bag_for_image
                    ]
                )
            ]
        )
    )
    ld.add_action(when_rviz2_starting)
    
    for paths, dirnames, filenames in os.walk(data_path.remap_dir_path):
        gimp_open_path = os.path.join(data_path.remap_dir_path, str(int(len(filenames) / 2)) + "_remap.pgm")
    gimp_grid_map = ExecuteProcess(
        cmd=[
            FindExecutable(name="gimp"),
            gimp_open_path
        ]
    )
    map2remap_node = Node(
        package="whill_navi2",
        executable="map2remap_node"
    )
    
    when_rviz2_exiting = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz2_node,
            on_exit=[map2remap_node, TimerAction(period=0.5, actions=[gimp_grid_map])]
        )
    )
    ld.add_action(when_rviz2_exiting)
    
    when_gimp_exiting = RegisterEventHandler(
        OnProcessExit(
            target_action=gimp_grid_map,
            on_exit=[
                TimerAction(period=0.5, actions=[Shutdown(reason="GIMP is over")]),
                
            ]
        )
    )
    ld.add_action(when_gimp_exiting)
    
    return ld