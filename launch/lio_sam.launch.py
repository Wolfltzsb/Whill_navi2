from whill_navi2.modules.ros2_launch_utils import (
    DataPath, os,
    get_file_path,
    LaunchDescription, Node, IncludeLaunchDescription,
    PythonLaunchDescriptionSource, GroupAction,
    DeclareLaunchArgument, LaunchConfiguration,
    ExecuteProcess, FindExecutable, IfCondition,
    RegisterEventHandler, OnExecutionComplete, 
    TimerAction, OnProcessExit, LogInfo, Shutdown,
    UnlessCondition
)

def generate_launch_description():
    
    data_path = DataPath()
    
    lio_sam_yaml_path = get_file_path("whill_navi2", "lio_sam_node_params.yaml")
    navsat_ekf_params_yaml_path = get_file_path("whill_navi2", "dual_ekf_navsat_params.yaml")
    mkdir_params_yaml_path = get_file_path("whill_navi2", "make_dir_node_params.yaml")
    lio_sam_rviz_path = get_file_path("whill_navi2", "lio_sam_launch.rviz")
    lio_sam_mapviz_path = get_file_path("whill_navi2", "lio_sam_launch.mvc")
    sensor_launch_path = get_file_path("whill_navi2", "sensor.launch.py")
    kuaro_whill_launch_path = get_file_path("whill_navi2", "kuaro_whill.launch.py")
    tf2_static_launch_path = get_file_path("whill_navi2", "tf2_static.launch.py")
    
    pcd_resolution = "0.2"
    with_data_recorded = "true"
    with_map_saved = "true"
    data_path.backup_data(os.path.basename(data_path.map_base_path))
    data_path.backup_data(os.path.basename(data_path.bag_base_path))
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument("with_data_recorded", default_value=with_data_recorded))
    with_data_recorded = LaunchConfiguration("with_data_recorded")
    ld.add_action(DeclareLaunchArgument("with_map_saved", default_value=with_map_saved))
    with_map_saved = LaunchConfiguration("with_map_saved")
    ld.add_action(DeclareLaunchArgument("pcd_resolution", default_value=pcd_resolution))
    pcd_resolution = LaunchConfiguration("pcd_resolution")

    # sensors
    ld.add_action(
        IncludeLaunchDescription(
        
            PythonLaunchDescriptionSource(sensor_launch_path),
            launch_arguments=[
                ("use_adis_imu", "false"),
                ("use_wit_imu", "false"),
                ("use_velodyne", "true"),
                ("use_ublox", "true"),
                ("use_hokuyo", "false"),
                ("use_web_camera", "false"),
                ("use_realsense_camera", "false"),
                ("use_zed_camera", "true")
            ]
        )
    )

    # kuaro whill
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kuaro_whill_launch_path),
            launch_arguments=[
                ("manual_mode", "true")
            ]
        )
    )
    
    # tf2 static
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tf2_static_launch_path),
            launch_arguments=[
                ("hokuyo_frame", "laser_front"),
                ("velodyne_frame", "velodyne"),
                ("adis_imu_frame", "imu_adis"),
                ("wit_imu_frame", "imu_wit"),
                ("whill_frame", "base_link"),
                ("zed_camera_frame", "zed_camera")
            ]
        )
    )
    # Frame of Aerial Photo
    ld.add_action(Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin_node",
        remappings=[
            ("fix", "/gnss/fix")
        ]
    ))
    
    # LIO-SAM
    ld.add_action(TimerAction(
        period=0.5,
        actions=[
            Node(
                package='lio_sam',
                executable='lio_sam_imuPreintegration',
                name='lio_sam_imuPreintegration',
                parameters=[
                    lio_sam_yaml_path
                ],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_imageProjection',
                name='lio_sam_imageProjection',
                parameters=[
                    lio_sam_yaml_path
                ],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_featureExtraction',
                name='lio_sam_featureExtraction',
                parameters=[
                    lio_sam_yaml_path
                ],
                output='screen'
            ),
            Node(
                package='lio_sam',
                executable='lio_sam_mapOptimization',
                name='lio_sam_mapOptimization',
                parameters=[
                    lio_sam_yaml_path,
                    {
                        "savePCDDirectory": data_path.pcd_map_dir_path,
                        "savePCD": with_map_saved
                    }
                ],
                output='screen'
            ),
            # Ekf_local and Navsat
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node_local',
                parameters=[
                    navsat_ekf_params_yaml_path
                ],
                remappings=[
                    ("odometry/filtered", "/odometry/filtered/local")
                ]
            ),
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                parameters=[
                    navsat_ekf_params_yaml_path,
                    {"use_odometry_yaw" : False}
                ],
                remappings=[
                    ("gps/fix", "/gnss/fix"),
                    ("gps/filtered", "/gnss/filtered"),
                    ("imu", "/zed/zed_node/imu/data_filtered"),
                    ("odometry/gps", "/odometry/gnss")
                ]
            )
        ]
    ))
    
    make_dir = Node(
        package='whill_navi2',
        executable='make_dir_node',
        parameters=[mkdir_params_yaml_path],
        output="screen"
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='lio_sam_rviz',
        arguments=['-d', lio_sam_rviz_path],
        output="screen"                            
    )
    # mapviz = Node(
    #     package="mapviz",
    #     executable="mapviz",
    #     name="lio_sam_mapviz",
    #     parameters=[{"config" : lio_sam_mapviz_path}]
    # )
    save_map = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "service", "call", "/lio_sam/save_map",
            "lio_sam/srv/SaveMap", 
            ["{", "resolution: ", pcd_resolution, ", destination: ", data_path.pcd_map_dir_path, "}"]
        ],
        output="screen",
        condition=IfCondition(with_map_saved)
    )
    
    ld.add_action(make_dir)
    
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=make_dir,
            on_exit=[
                # Record Data
                TimerAction(
                    period=0.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                FindExecutable(name='ros2'),
                                'bag', 'record', '--regex', '.*(imu|odometry|fix|velodyne|gnss|tf|ublox|lio_sam|).*', '-o', data_path.bag_file_path
                            ],
                            output="screen",
                            condition=IfCondition(with_data_recorded)
                        )
                    ]
                ),
                # Record Waypoint
                Node(
                    package="waypoint_pkg",
                    executable="data2waypoint",
                    name='waypoint_maker',
                    parameters=[{
                        'waypoint_file' : data_path.waypoint_file_path,
                        "waypoint_distance" : 5.0,
                        "yaw_deg_thresh" : 10.0,
                        "deg_chord" : 0.8
                    }],
                    arguments=['--ros-args', '--log-level', 'info'],
                    remappings=[
                        ("imu", "/zed/zed_node/imu/data_filtered"),
                        ("fix", "/gnss/fix"),
                        ("odometry", "/odometry/gnss")
                    ],
                    output="screen"
                ),
                rviz
            ]
        )
    ))
    
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[TimerAction(
                period=0.5,
                actions=[
                    save_map
                ]
            )]
        )
    ))
    
    ld.add_action(RegisterEventHandler(
        OnExecutionComplete(
            target_action=save_map,
            on_completion=[
                LogInfo(msg="Pcd map is saved successfully. And then shutdown Launch Process."),
                TimerAction(
                    period=3.0,
                    actions=[
                        Shutdown()
                    ]
                )
            ]
        ),
        condition=IfCondition(with_map_saved)  
    ))
    
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[TimerAction(
                period=0.5,
                actions=[
                    Shutdown()
                ]
            )]
        ),
        condition=UnlessCondition(with_map_saved)
    ))
    

    return ld