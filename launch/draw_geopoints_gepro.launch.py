from whill_navi2.modules.ros2_launch_utils import (
    LaunchDescription, Node, DataPath,
    DeclareLaunchArgument, LaunchConfiguration,
    GroupAction, RegisterEventHandler,
    OnProcessExit, Shutdown, os,
    get_file_path
)


def generate_launch_description():
    
    # place_name = "Kansai University Rinpukan"
    # initialize_origin_frame_node = "initialize_origin_frame"
    # origin_geolocation_topic = "/origin/fix"
    # map_frame = "map"
    # tile_map_frame = "origin"
    
    # mapviz_config_path = get_file_path("whill_navi2", "draw_topological_map_launch.mvc")
    make_dir_params = get_file_path("whill_navi2", "make_dir_node_params.yaml")
    
    ld = LaunchDescription()
    
    # ld.add_action(DeclareLaunchArgument(name="place_name", default_value=place_name))
    # place_name = LaunchConfiguration("place_name")
    
    # Make Directory
    make_dir_node = Node(
        package="whill_navi2",
        executable="make_dir_node",
        parameters=[make_dir_params]
    )
    ld.add_action(make_dir_node)
    
    # Get Geolocation and Save topological map
    ld.add_action(GroupAction(actions=[
        Node(
            package="whill_navi2",
            executable="save_topological",
            parameters=[{
                "file_path" : os.path.join(DataPath().waypoint_base_path, "GeoPoints", "topological_map.yaml")
            }]
        ),
        Node(
            package="whill_navi2",
            executable="google_earth_node"
        )        
    ]))
        
    return ld