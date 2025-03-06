import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from whill_navi2.modules.ros2_launch_utils import DataPath, get_param_dict, get_file_path, DeclareLaunchArgument

def generate_launch_description():
    
    data_path = DataPath()
    hdl_localization_config_path = get_file_path("whill_navi2", "hdl_localization_params.yaml")
    globalmap_server_config_dict = get_param_dict(hdl_localization_config_path, "globalmap_server_node")
    
    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument(name="globalmap_pcd", default_value=os.path.join(data_path.pcd_map_dir_path, "glim.pcd")))
    globalmap_pcd = LaunchConfiguration("globalmap_pcd")
    ld.add_action(DeclareLaunchArgument(name="point_cloud_topic", default_value="velodyne_points"))
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    ld.add_action(DeclareLaunchArgument(name="gridmap_topic", default_value="map"))
    gridmap_topic = LaunchConfiguration("gridmap_topic")
    ld.add_action(DeclareLaunchArgument(name="imu_topic", default_value="adis/imu/data"))
    imu_topic = LaunchConfiguration("imu_topic")
    ld.add_action(DeclareLaunchArgument(name="plot_estimation_errors", default_value="False", choices=["False", "True", "false", "true"]))
    ld.add_action(DeclareLaunchArgument(name="use_sim_time", default_value="False", choices=["False", "True", "false", "true"]))
    use_sim_time = LaunchConfiguration("use_sim_time")
    plot_estimation_errors = LaunchConfiguration("plot_estimation_errors")
    ld.add_action(DeclareLaunchArgument(name="globalmap_resolution", default_value="0.2"))
    globalmap_resolution = LaunchConfiguration("globalmap_resolution")
    ld.add_action(DeclareLaunchArgument(name="scaning_resolution", default_value="0.1"))
    scaning_resolution = LaunchConfiguration("scaning_resolution")
    ld.add_action(DeclareLaunchArgument(name="send_tf_transforms", default_value="True", choices=["False", "True", "false", "true"]))
    send_tf_transforms = LaunchConfiguration("send_tf_transforms")
    
    ld.add_action(DeclareLaunchArgument(name="init_pos_x", default_value="0.0"))
    init_pos_x = LaunchConfiguration("init_pos_x")
    ld.add_action(DeclareLaunchArgument(name="init_pos_y", default_value="0.0"))
    init_pos_y = LaunchConfiguration("init_pos_y")
    ld.add_action(DeclareLaunchArgument(name="init_pos_z", default_value="0.0"))
    init_pos_z = LaunchConfiguration("init_pos_z")
    ld.add_action(DeclareLaunchArgument(name="init_ori_x", default_value="0.0"))
    init_ori_x = LaunchConfiguration("init_ori_x")
    ld.add_action(DeclareLaunchArgument(name="init_ori_y", default_value="0.0"))
    init_ori_y = LaunchConfiguration("init_ori_y")
    ld.add_action(DeclareLaunchArgument(name="init_ori_z", default_value="0.0"))
    init_ori_z = LaunchConfiguration("init_ori_z")
    ld.add_action(DeclareLaunchArgument(name="init_ori_w", default_value="1.0"))
    init_ori_w = LaunchConfiguration("init_ori_w")
    
    ld.add_action(DeclareLaunchArgument(name="namespace", default_value=""))
    namespace = LaunchConfiguration("namespace")
    
    hdl_global_localization_node_params = get_param_dict(hdl_localization_config_path, "hdl_global_localization_node")
    hdl_global_localization_node_params["config_path"] = os.path.join(data_path.params_base_dir_path, "hdl_global_localization_config")
    hdl_global_localization_node_params["downsample_resolution"] = globalmap_resolution
    hdl_global_localization_node = Node(
        package="hdl_global_localization",
        executable="hdl_global_localization_node",
        parameters=[hdl_global_localization_node_params],
        namespace=namespace,
        remappings=[
            # ((namespace, "bbs/gridmap"), (namespace, "/hdl_global_localization/bbs/gridmap")),
            ((namespace, "/bbs/gridmap"), (gridmap_topic)),
            ((namespace, "/query"), (namespace, "/hdl_global_localization/query"))
        ],
        output="screen"
    )
    
    hdl_localization_config_dict = get_param_dict(hdl_localization_config_path, "hdl_localization_node")
    hdl_localization_config_dict["use_sim_time"] = use_sim_time
    hdl_localization_config_dict["init_pos_x"] = init_pos_x
    hdl_localization_config_dict["init_pos_y"] = init_pos_y
    hdl_localization_config_dict["init_pos_z"] = init_pos_z
    hdl_localization_config_dict["init_ori_x"] = init_ori_x
    hdl_localization_config_dict["init_ori_y"] = init_ori_y
    hdl_localization_config_dict["init_ori_z"] = init_ori_z
    hdl_localization_config_dict["init_ori_w"] = init_ori_w
    hdl_localization_config_dict["downsample_resolution"] = scaning_resolution
    hdl_localization_config_dict["send_tf_transforms"] = send_tf_transforms
    hdl_localization_node = Node(
        package="hdl_localization",
        executable="hdl_localization_node",
        name="hdl_localization_node",
        namespace=namespace,
        parameters=[hdl_localization_config_dict],
        remappings=[
            # Subscribe
            ((namespace, "/velodyne_points"), point_cloud_topic),
            ((namespace, "/imu_data"), imu_topic),
            # Publish
            ((namespace, "/odom"), (namespace, "/hdl_localization/odom")),
            ((namespace, "/relocalize"), (namespace, "/hdl_localization/relocalize")),
            ((namespace, "/query"), (namespace, "/hdl_global_localization/query"))
        ],
        output="screen"
    )
    ld.add_action(hdl_localization_node)
    ld.add_action(hdl_global_localization_node)    
    
    globalmap_server_config_dict["globalmap_pcd"] = globalmap_pcd
    globalmap_server_config_dict["downsample_resolution"] = globalmap_resolution
    globalmap_server_node = Node(
        package="hdl_localization",
        executable="globalmap_server_node",
        name="globalmap_server_node",
        namespace=namespace,
        parameters=[globalmap_server_config_dict],
        output="screen"
    )
    ld.add_action(globalmap_server_node)

    plot_status = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            os.path.join(
                get_package_share_directory("hdl_localization"),
                "scripts", "plot_status.py"
            )
        ],
        condition=IfCondition(plot_estimation_errors)
    )
    ld.add_action(plot_status)
    # record_screen = ExecuteProcess(
    #     cmd=[
    #         FindExecutable(name="ffmpeg"),
    #         "-video_size 1920x1080 -framerate 25 -f x11grab -i :1",
    #         os.path.join(DataPath().base_path, "record_navigation.mp4")
    #     ]
    # )
    # ld.add_action(record_screen)

    return ld
