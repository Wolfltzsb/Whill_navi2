from whill_navi2.modules.ros2_launch_utils import (
    get_file_path,
    LaunchDescription, Node, DeclareLaunchArgument,
    LaunchConfiguration
)

def generate_launch_description():
    
    wt901_params_yaml_path = get_file_path("whill_navi2", "wt901_params.yaml")

##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################    
    
    ld = LaunchDescription()
            
    wit901_node=Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        name="witmotion",
        parameters=[
            wt901_params_yaml_path
        ]
    )
    ld.add_action(wit901_node)
        
    return ld