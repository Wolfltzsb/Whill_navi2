import os, shutil, glob, datetime, yaml

# Ros Launch Modules
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, \
                            ExecuteProcess, RegisterEventHandler, LogInfo, \
                            TimerAction, GroupAction, Shutdown, EmitEvent, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, OrSubstitution, TextSubstitution, Command
from launch.substitution import Substitution
from launch.event_handlers import OnExecutionComplete, OnProcessStart, OnShutdown, OnProcessExit
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from launch.actions.shutdown_action import ShutdownEvent
from rclpy.time import Time
from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader

def split_bag(bag_path : str, split_interval_seconds : float):
    # 转换秒到纳秒
    split_interval = int(split_interval_seconds * 1e9)

    # 设置存储和转换选项
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', 
                                         output_serialization_format='cdr')

    # 创建一个顺序读取器
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # 读取原始的metadata.yaml文件
    metadata_path = os.path.join(bag_path, 'metadata.yaml')
    with open(metadata_path, 'r') as f:
        metadata = yaml.safe_load(f)

    # 获取bag的开始时间和持续时间
    start_time = metadata['rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch']
    duration = metadata['rosbag2_bagfile_information']['duration']['nanoseconds']
    end_time = start_time + duration

    # 计算需要分割的次数
    split_count = (duration + split_interval - 1) // split_interval

    for i in range(split_count):
        split_start = start_time + i * split_interval
        split_end = min(split_start + split_interval, end_time)
        
        if (split_end - split_start) < int(1e9):
            break

        # 创建新的metadata
        new_metadata = metadata.copy()
        new_metadata['rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch'] = split_start
        new_metadata['rosbag2_bagfile_information']['duration']['nanoseconds'] = split_end - split_start
        new_metadata['rosbag2_bagfile_information']['files'][0]["starting_time"]['nanoseconds_since_epoch'] = split_start
        new_metadata['rosbag2_bagfile_information']['files'][0]['duration']['nanoseconds'] = split_end - split_start

        # 生成新的metadata文件名
        new_metadata_filename = "metadata_split_{}.yaml".format(i)
        new_metadata_path = os.path.join(bag_path, new_metadata_filename)

        # 将新的metadata写入原始bag目录
        with open(new_metadata_path, 'w+') as f:
            yaml.dump(new_metadata, f)

    print(f"Bag has been virtually split into {split_count} parts. New metadata files are in the original bag directory.")

def get_param_dict(yaml_file_path : str, node_name : str) -> dict:
    '''
        Get the parameters in a yaml file
        
        Args:
        - yaml_path: the path of yaml file
        - node_name: node name specified in yaml file as a key
        
        Return:
        - data: Dict{key1 : value1, key2 :value2, ....}
    '''      
    with open(yaml_file_path) as f:
        data = yaml.safe_load(f)[node_name]["ros__parameters"]
    return data

def get_file_path(pkg_name : str, file_name : str) -> str :
    '''
        Get file path in a given package path
        
        Args:
        - pkg_name: a package name of ROS
        - file_name: file name
                    
        Return:
        - file_path: A file path by a given file name
    '''           
    config_path = os.path.join(get_package_share_directory(pkg_name), "config")
    launch_path = os.path.join(get_package_share_directory(pkg_name), "launch")
    
    if "launch.py" in file_name:
        for file_path in glob.glob(os.path.join(launch_path, "**"), recursive=True):
            if file_name == os.path.basename(file_path):
                return file_path
            
        print("Unable to found {} in '{}'!".format(file_name, launch_path))
        exit(1)
    else:
        for file_path in glob.glob(os.path.join(config_path, "**"), recursive=True):
            if file_name == os.path.basename(file_path):
                return file_path
            
        print("Unable to found {} in '{}'!".format(file_name, config_path))
        exit(1)
    
class DataPath:
    '''
        Help developer to access data path easily by creating this instance
    '''

    def __init__(self):
        '''
            Find the yaml file which has the information about the path and file name of DATA. And then initialize the Attritubes of this class with these information.
        ''' 
        mkdir_params_yaml_path = get_file_path('whill_navi2', 'make_dir_node_params.yaml')
        mkdir_node_params_dict = get_param_dict(mkdir_params_yaml_path, 'make_dir_node')
        self.base_path = os.path.join(
            os.environ['HOME'],
            "Documents",
            'full_data',
            mkdir_node_params_dict['place_dir'],
            str(mkdir_node_params_dict['date_dir'])
        )
        self.bag_base_path = os.path.join(self.base_path, mkdir_node_params_dict['bag_dir'])
        self.map_base_path = os.path.join(self.base_path, mkdir_node_params_dict['map_dir'])
        self.waypoint_base_path = os.path.join(self.base_path, mkdir_node_params_dict['waypoint_dir'])
        self.branchpoint_base_path = os.path.join(self.base_path, mkdir_node_params_dict['branchpoint_dir'])
        
        self.bag_dir_path = os.path.join(self.bag_base_path, mkdir_node_params_dict['bag_dir'])
        self.map_dir_path = os.path.join(self.map_base_path, mkdir_node_params_dict['map_dir'])
        self.remap_dir_path = os.path.join(self.map_base_path, mkdir_node_params_dict['remap_dir'])
        self.pcd_map_dir_path = os.path.join(self.map_base_path, mkdir_node_params_dict['pcd_map_dir'])
        self.waypoint_dir_path = os.path.join(self.waypoint_base_path, mkdir_node_params_dict['waypoint_dir'])
        self.rewaypoint_dir_path = os.path.join(self.waypoint_base_path, mkdir_node_params_dict['rewaypoint_dir'])
        
        self.bag_name = mkdir_node_params_dict['bag_name']       
        self.map_name = mkdir_node_params_dict['map_name']
        self.remap_name = mkdir_node_params_dict['remap_name']
        self.waypoint_name = mkdir_node_params_dict['waypoint_name']
        self.rewaypoint_name = mkdir_node_params_dict['rewaypoint_name']
        
        self.bag_file_path = os.path.join(self.bag_dir_path, self.bag_name)
        self.map_file_path = os.path.join(self.map_dir_path, self.map_name)
        self.remap_file_path = os.path.join(self.remap_dir_path, self.remap_name)
        self.waypoint_file_path = os.path.join(self.waypoint_dir_path, self.waypoint_name)
        self.rewaypoint_file_path = os.path.join(self.rewaypoint_dir_path, self.rewaypoint_name)
        
        self.params_base_dir_path = os.path.join(get_package_share_directory("whill_navi2"), "config", "params")
    
    def backup_data(self, data_type : str):
        '''
            Backup Data

            Args:
            - data_type: a string to indicate the data type you want to backup.
                - data_type = One of ["waypoint", "bag", "map" and "branchpoint"]
        ''' 

        if data_type != os.path.basename(self.bag_base_path) \
            and data_type != os.path.basename(self.map_base_path) \
            and data_type !=os.path.basename(self.waypoint_base_path) \
            and data_type !=os.path.basename(self.branchpoint_base_path) \
        :
            print("Please give a data type which is {}, {}, {}, {} !!!".format(
                os.path.basename(self.bag_base_path),
                os.path.basename(self.map_base_path),
                os.path.basename(self.waypoint_base_path),
                os.path.basename(self.branchpoint_base_path)
            ))
            return
        
        backup_data_path = os.path.join(self.base_path, data_type)
        backup_target_path = os.path.join(backup_data_path, "Backup")
        backup_source_file_list = glob.glob(os.path.join(backup_data_path, "*"))
        
        if (len(glob.glob(os.path.join(backup_data_path, "**"), recursive=True)) - len(backup_source_file_list)) <= 1 :
            return
        
        for file in backup_source_file_list:
            if "Backup" in file:
                backup_source_file_list.remove(file)
        
        target_path = os.path.join(backup_target_path, datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        os.makedirs(target_path)
        for file_path in backup_source_file_list:
            if "Backup" in file_path:
                continue
            shutil.move(
                file_path,
                target_path
            )
            os.makedirs(os.path.dirname(file_path))
        
    def get_rewapypoint_path(self):
        '''
            Check if any file is in the "rewaypoint" directory. If ture, retrun file path from "rewaypoint" directory. Else, read from "waypoint" directroy.
            
            Return:
            - [read_path, write_path]
                - read_path: the path which should be read
                - write_path: the path which should be writen
        '''        
        files = glob.glob(os.path.join(self.rewaypoint_dir_path, "*"))
        if len(files) == 0:
            read_path = self.waypoint_file_path
            write_path = os.path.join(self.rewaypoint_dir_path, '1_' + self.rewaypoint_name)
        else:
            path_num = len(files)
            read_path = os.path.join(self.rewaypoint_dir_path, str(path_num) + '_' + self.rewaypoint_name)
            write_path = os.path.join(self.rewaypoint_dir_path, str(path_num + 1) + '_' + self.rewaypoint_name)
        
        return [read_path, write_path]
                
