from whill_navi2.modules.ros2_launch_utils import (
    get_file_path,
    Node, LaunchDescription, LaunchConfiguration,
    DeclareLaunchArgument
)

def generate_launch_description():
    
    adis16465_params_yaml_path = get_file_path("whill_navi2", 'adis16465_node_params.yaml')
    adis16465_node_ns = "adis"

##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################

    ld = LaunchDescription()
        
    imu_node = Node(
        package="adi_driver",                                           
        executable="adis16465_node",
        name="adis16465_node",
        parameters=[adis16465_params_yaml_path],
        remappings=[
            ("imu/data_raw", "/{}/imu/data_raw".format(adis16465_node_ns))
        ],
        output="screen"                                                 
    )
    ld.add_action(imu_node)
    
    imu_filter_node = Node(
        package="imu_filter_madgwick",                                  
        executable="imu_filter_madgwick_node",                          
        name="adis_imu_filter_node",
        parameters=[adis16465_params_yaml_path],
        remappings=[
            ("imu/data_raw", "/{}/imu/data_raw".format(adis16465_node_ns)),
            ("imu/data", "/{}/imu/data".format(adis16465_node_ns)),
            ("imu/mag", "/{}/imu/mag".format(adis16465_node_ns))
        ]
    )
    ld.add_action(imu_filter_node)

    return ld