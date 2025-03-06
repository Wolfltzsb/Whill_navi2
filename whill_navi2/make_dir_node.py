import rclpy
from rclpy.node import Node
import os, sys

#
# データ保存用ディレクトリ作成用Node
# Path_Tree
# 例：
# $HOME/Documents
# └── full_data
#     └── place
#         └── datetime
#             ├── bag
#                   ├── Backup
#                   ├── bag
#             ├── waypoint
#                   ├── Backup
#                   ├── waypoint
#                   ├── rewaypoint
#             ├── map
#                   ├── Backup
#                   ├── map
#                   ├── remap
#                   ├── pcd_map
#             ├── branchpoint

class MakeDir(Node):
    def __init__(self):
        super().__init__("make_dir_node")

        # Intialize variables
        self._prefix_path_ = str()
        self._suffix_paths_dict_ = {str : str}
        self._suffix_path_key_list_ = [str]
           
        # Declare Parameters
        place = self.declare_parameter("place_dir", "nakanoshima").get_parameter_value().string_value
        date = self.declare_parameter("date_dir", "date_today").get_parameter_value().string_value
        map_dir = self.declare_parameter("map_dir", "map").get_parameter_value().string_value
        remap_dir = self.declare_parameter("remap_dir", "remap").get_parameter_value().string_value
        pcd_dir = self.declare_parameter("pcd_map_dir", "pcd").get_parameter_value().string_value
        bag_dir = self.declare_parameter("bag_dir", "bag").get_parameter_value().string_value
        waypoint_dir = self.declare_parameter("waypoint_dir", "waypoint").get_parameter_value().string_value
        rewaypoint_dir = self.declare_parameter("rewaypoint_dir", "rewaypoint").get_parameter_value().string_value
        branchpoint_dir = self.declare_parameter("branchpoint_dir", "branchpoint").get_parameter_value().string_value

        # Set variables
        self._prefix_path_ = os.path.join(
            os.path.expanduser("~"),
            "Documents",
            'full_data',
            place,
            date,
        )
        self._dirs_dict_ = {
            map_dir: [map_dir, remap_dir, pcd_dir, "Backup"], 
            bag_dir: [bag_dir, "Backup"], 
            waypoint_dir: [waypoint_dir, rewaypoint_dir, "GeoPoints", "Backup"], 
            branchpoint_dir: [branchpoint_dir, "Backup"]
        }
        self._dir_index_ = [
            map_dir,
            bag_dir,
            waypoint_dir,
            branchpoint_dir
        ]

        self.make_dir()

        # Debug
        # print(self._prefix_path_)

    def make_dir(self):
        
        for dir_index in self._dir_index_:
            for dir in self._dirs_dict_[dir_index]:
                full_path = os.path.join(
                    self._prefix_path_,
                    dir_index,
                    dir
                )
                if not os.path.exists(full_path):
                    os.makedirs(full_path)
                    # Debug
                    # print(full_path)
                else:
                    self.get_logger().warn("Path exists!" + full_path)
                    continue
        
        self.get_logger().info("Directories are generated successfully.")

def main():
    rclpy.init(args=sys.argv)
    # rclpy.spin(MakeDir)
    # rclpy.shutdown()
    node = MakeDir()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
