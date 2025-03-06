from whill_navi2.modules.ros2_launch_utils import (
    LaunchDescription, Node, DataPath,
    DeclareLaunchArgument, LaunchConfiguration,
    GroupAction, RegisterEventHandler,
    OnProcessExit, Shutdown, os,
    get_file_path, ExecuteProcess, FindExecutable
)

def generate_launch_description():
    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument(name="T_utm_topomap_file_path", default_value=os.path.join(DataPath().waypoint_base_path, "GeoPoints", "T_utm_topomap.yaml")))
    T_utm_topomap_file_path = LaunchConfiguration("T_utm_topomap_file_path")
    ld.add_action(DeclareLaunchArgument(name="T_topomap_liomap_file_path", default_value=os.path.join(DataPath().waypoint_base_path, "GeoPoints", "T_topomap_liomap.yaml")))
    T_topomap_liomap_file_path = LaunchConfiguration("T_topomap_liomap_file_path")
    ld.add_action(DeclareLaunchArgument(name="front_speed", default_value="1.0"))
    front_speed = LaunchConfiguration("front_speed")
    ld.add_action(DeclareLaunchArgument(name="turn_speed", default_value="1.0"))
    turn_speed = LaunchConfiguration("turn_speed")
    
    map_frame = "lio_map"
    
    lio_gnss_localization_node = Node(
        package="whill_navi2",
        executable="lio_gnss_localization_node",
        name="lio_gnss_localization_node",
        parameters=[{
            "T_utm_topomap_file": T_utm_topomap_file_path,
            "T_topomap_liomap_file": T_topomap_liomap_file_path,
            "map_frame": map_frame,
            "buffer_len": 30,
            "translation_tollerance": 0.1,
            "rotation_threshold": 0.01,
            # "front_speed": front_speed,
            # "turn_speed": turn_speed,
            "snr_threshold": 27.0,
            # "oi_threshold": 5.0,
            "hdop_threshold": 0.8,
            "high_elevation_num_threshold": 13
        }],
        output="screen"
    )
    ld.add_action(lio_gnss_localization_node)
    imu_points_sync_node = Node(
        package="whill_navi2",
        executable="imu_points_sync_node",
        name="imu_points_sync_node"
    )
    ld.add_action(imu_points_sync_node)
    
    glim_odometry_node = Node(
        package="glim_ros",
        executable="glim_rosnode",
        name="glim_rosnode",
        arguments=[
            "--ros-args", "-p", ("dump_path:=", os.path.join(DataPath().pcd_map_dir_path, "tmp_dump")),
            "-p", "auto_quit:=true", "-p", "config_path:={}".format(os.path.join(DataPath().params_base_dir_path, "glim_for_localization_config"))
        ],
        output="screen"
    )
    ld.add_action(glim_odometry_node)
    
    record_odometry_when_running = Node(
        package="waypoint_pkg",
        executable="waypoint_recorder",
        name="record_odometry_when_running",
        parameters=[{
            "waypoint_file_path": os.path.join(DataPath().waypoint_dir_path, "running_real_odometry.yaml"),
            "source_frame": "topomap",
            "target_frame": "base_link",
            "delta_distance": 1.0,
            "delta_yaw": 10.0,
            "delta_chrod": 0.5,
            "with_rviz": False,
            "from_topic": False,
            "from_gnss": False,
            # "topic_name": "/odometry/filtered/global"
        }]
    )
    ld.add_action(record_odometry_when_running)
    record_gnss_when_running = Node(
        package="waypoint_pkg",
        executable="waypoint_recorder",
        name="record_gnss_when_running",
        parameters=[{
            "waypoint_file_path": os.path.join(DataPath().waypoint_dir_path, "running_gnss.yaml"),
            "source_frame": "topomap",
            "target_frame": "base_link",
            "delta_distance": 1.0,
            "delta_yaw": 10.0,
            "delta_chrod": 0.5,
            "with_rviz": False,
            "from_topic": True,
            "from_gnss": True,
            "topic_name": "/gnss/fix"
        }]
    )
    ld.add_action(record_gnss_when_running)
    record_original_lio_when_running = Node(
        package="waypoint_pkg",
        executable="waypoint_recorder",
        name="record_original_lio_when_running",
        parameters=[{
            "waypoint_file_path": os.path.join(DataPath().waypoint_dir_path, "running_original_lio.yaml"),
            "source_frame": map_frame,
            "target_frame": "base_link",
            "delta_distance": 1.0,
            "delta_yaw": 10.0,
            "delta_chrod": 0.5,
            "with_rviz": False,
            "from_topic": False,
            "from_gnss": False,
            # "topic_name": "/glim_rosnode/odom"
        }]
    )
    ld.add_action(record_original_lio_when_running)
    
    record_data_bag = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "bag", "record", 
            "--regex", ".*(odom|nmea)*.",
            "--output", os.path.join(DataPath().bag_dir_path, "Backup", "running_for_evo_eval")
        ]
    )
    ld.add_action(record_data_bag)
    # record_screen = ExecuteProcess(
    #     cmd=[
    #         FindExecutable(name="ffmpeg"),
    #         "-video_size 1920x1080 -framerate 25 -f x11grab -i :1",
    #         os.path.join(DataPath().base_path, "record_navigation.mp4")
    #     ]
    # )
    # ld.add_action(record_screen)

    
    return ld