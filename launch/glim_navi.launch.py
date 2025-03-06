from whill_navi2.modules.ros2_launch_utils import (
    Node, LaunchConfiguration, LaunchDescription, IncludeLaunchDescription,
    DataPath, get_file_path, PythonLaunchDescriptionSource, ExecuteProcess,
    FindExecutable, GroupAction, RegisterEventHandler, OnProcessStart,
    TimerAction, DeclareLaunchArgument, OnExecutionComplete
)


def generate_launch_description():
    
    ld = LaunchDescription()
    data_path = DataPath()
    
    ld.add_action(DeclareLaunchArgument(name="initial_waypoint_number", default_value="0"))
    initial_waypoint_number = LaunchConfiguration("initial_waypoint_number")
    
    # Sensor
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_file_path("whill_navi2", "sensor.launch.py")),
        launch_arguments=[
            ("use_velodyne", "True"),
            ("use_zed_camera", "True"),
            ("use_adis_imu", "True"),
            ("use_m20_lidar", "True")
        ]
    )
    
    whill_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_file_path("whill_navi2", "kuaro_whill.launch.py"))
    )

    hdl_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_file_path("whill_navi2", "hdl_locliazation.launch.py"))
    )
    
    # navigation
    navigation_params_path = get_file_path("whill_navi2", "nav2_params.yaml")
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_file_path("whill_navi2," "navigation.launch.py")),
        launch_arguments=[
            ("params_file", navigation_params_path),
            ("map_yaml_filename", data_path.remap_file_path + ".yaml")
        ]
    )
    
    # odom_ekf_node
    ekf_filter_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        parameters=[get_file_path("whill_navi2", "dual_ekf_navsat_params.yaml")],
        remappings=[("/odometry/filtered", "/odometry/filtered/local")]
    )
    
    # rviz
    glim_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", get_file_path("whill_navi2", "glim_navi.rviz")]
    )
    ld.add_action(glim_rviz_node)
    
    # When rviz is starting
    origin_waypoint_path, rewaypoint_path = data_path.get_rewapypoint_path()
    whill_navi2_node = Node(
        package="whill_navi2",
        executable="whill_navi2_node",
        parameters=[{
            "waypoint_read_file" : rewaypoint_path,
            "initial_waypoint_number" : initial_waypoint_number,
            "is_loop" : False,
            "loop_limit" : 2,
            "voxel_resolution" : 0.1,
            "stop_distance" : 1.0
        }]
    )
    
    when_rviz_starting = RegisterEventHandler(
        OnProcessStart(
            target_action=glim_rviz_node,
            on_start=[TimerAction(period=0.5, actions=[sensor_launch, whill_launch])]
        )
    )
    ld.add_action(when_rviz_starting)
    
    when_sensor_starting = RegisterEventHandler(
        OnExecutionComplete(
            target_action=sensor_launch,
            on_completion=[TimerAction(period=1.5, actions=[ekf_filter_local_node])]
        )
    )
    ld.add_action(when_sensor_starting)
    
    when_ekf_starting = RegisterEventHandler(
        OnProcessStart(
            target_action=ekf_filter_local_node,
            on_start=[TimerAction(period=0.1, actions=[hdl_localization_launch])]
        )
    )
    ld.add_action(when_ekf_starting)
    
    when_hdl_starting = RegisterEventHandler(
        OnExecutionComplete(
            target_action=hdl_localization_launch,
            on_completion=[TimerAction(period=5.0, actions=[navigation_launch])]
        )
    )
    ld.add_action(when_hdl_starting)
    
    when_navigation_starting = RegisterEventHandler(
        OnExecutionComplete(
            target_action=navigation_launch,
            on_completion=[TimerAction(period=2.0, actions=[whill_navi2_node])]
        )
    )
    ld.add_action(when_navigation_starting)
    
    return ld