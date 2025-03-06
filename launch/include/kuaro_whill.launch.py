from whill_navi2.modules.ros2_launch_utils import (
    get_file_path, get_param_dict,
    Node, LaunchDescription, DeclareLaunchArgument,
    LaunchConfiguration, IfCondition, ExecuteProcess,
    FindExecutable, UnlessCondition, LogInfo, 
    RegisterEventHandler, OnProcessExit, OnProcessStart,
    TimerAction
)

def generate_launch_description():
    
    whill_params_yaml_path = get_file_path("whill_navi2", "ros2_whill_params.yaml")
    
    # fm, fa, fd, rm, ra, rd, tm, ta, td  :config
    # 8   10  40  8   10  40  8   10  40  :min
    # ~   ~   ~   ~   ~   ~   ~   ~   ~
    # 60  90  160 30  50  90  35  60  160 :max

##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(name="publish_tf", default_value="false"))
    publish_tf = LaunchConfiguration("publish_tf")
        
    # Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',                                # パッケージの名前
        executable='robot_state_publisher',                             # 実行ファイルの名前
        name='robot_state_publisher',                                   # ノードの名前
        arguments=[get_file_path("whill_navi2", "modelc.urdf.xacro")],  # Nodeに与える引数、
        remappings=[('joint_states', 'whill/states/joint_state')],      # トピックのremap
        output='screen'                                                 # ログをコンソール画面に出力する
    )
    ld.add_action(robot_state_publisher_node)
    # Parameter YAML file: config/param/ros2_whill_params.yaml
    ros2_whill_params = get_param_dict(whill_params_yaml_path, "whill/ros2_whill")
    ros2_whill_params["publish_tf"] = publish_tf
    ros2_whill_node = Node(
        package='ros2_whill',
        executable='ros2_whill_node',
        name='ros2_whill',
        output='screen',
        namespace='whill',
        remappings=[
            ("odom", "odometry")
        ],
        parameters=[ros2_whill_params]
    )
    ld.add_action(ros2_whill_node)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        remappings=[
            ("/joy", "/joy")
        ]
    )
    ld.add_action(joy_node)
    # Parameter YAML file: config/param/whill_joy2_params.yaml
    whill_joy_node = Node(
        package='ros2_whill',
        executable='whill_joy',
        name="whill_joy",
        output='screen',
        parameters=[
            whill_params_yaml_path
        ],
        remappings=[
            ('/joy_in', '/joy'),
            ('/joy_out', '/whill/controller/joy'),
            ('/set_speed_profile_srv', '/whill/set_speed_profile_srv'),
            ('/set_power_srv', '/whill/set_power_srv')
        ],
    )
    ld.add_action(whill_joy_node)
    # ld.add_action(whill_initial_speed)
    
    speed_config = get_param_dict(whill_params_yaml_path, "whill_joy")
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=whill_joy_node,
            on_exit=[
                LogInfo(msg=[
                    "Max Speed Forward: {} m/s, Max Speed Trun: {} m/s".format(
                        speed_config["speed_profile_initial"][0] * 0.1 * 1000 / 3600, 
                        speed_config["speed_profile_initial"][6] * 0.1 * 1000 / 3600
                    )
                ])
            ]
        )
    ))            
    
    return ld
