from whill_navi2.modules.ros2_launch_utils import (
    get_file_path, get_param_dict,
    LaunchDescription, Node, IfCondition, os,
    LaunchConfiguration, DeclareLaunchArgument
)

def generate_launch_description():

    f9p_params_yaml_path = get_file_path("whill_navi2", "zed_f9p_node_params.yaml")
    f9p_params_data = get_param_dict(f9p_params_yaml_path, "ublox/zed_f9p_node")
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################

    ld = LaunchDescription()

    # Declare nodes
    zed_f9p_node = Node(
        package="zed_f9p",
        executable="zed_f9p_node",
        name="zed_f9p_node",
        namespace="ublox",
        parameters=[f9p_params_yaml_path],
    )
    ld.add_action(zed_f9p_node)
    ntrip_node = Node(
        package="zed_f9p",
        executable="ntrip_client_node",
        parameters=[f9p_params_yaml_path],
        remappings=[
            ("rtcm", "/ublox/rtcm"),
            ("nmea_gga", "/ublox/nmea_gga")            
        ],
        output="both",
        condition=IfCondition(str(f9p_params_data['use_rtk']))
    )
    ld.add_action(ntrip_node)
    navsat_node = Node(
        package="nmea_navsat_driver",
        executable="nmea_topic_driver",
        name="nmea_topic_driver",
        remappings=[
            ("fix", "/gnss/fix"),
            ("nmea_sentence", "/ublox/nmea")
        ],
        output="both"
    )
    ld.add_action(navsat_node)
   
    return ld
