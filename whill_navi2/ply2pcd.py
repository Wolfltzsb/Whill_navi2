import open3d, os, shutil, sys
import rclpy
from rclpy.node import Node
import rclpy.node

class Ply2Pcd(Node):
    
    def __init__(self):
        super().__init__("ply2pcd")
        
        self.ply_dir_path = self.declare_parameter("ply_dir_path", os.path.join(os.environ["HOME"], "Documents")).get_parameter_value().string_value
        self.pcd_dir_path = self.declare_parameter("pcd_dir_path", os.path.join(os.environ["HOME"], "Documents")).get_parameter_value().string_value
        self.auto_clean_tmp = self.declare_parameter("auto_clean_tmp", "False").get_parameter_value().bool_value
        
        self.create_timer(0.1, self.main_callback)
        
    
    def main_callback(self):
        if not os.path.exists(self.pcd_dir_path):
            self.get_logger().warn("Couldn't find pcd directory path which is {}, please check the path".format(self.pcd_dir_path))
            return
        elif not os.path.exists(self.ply_dir_path):
            self.get_logger().warn("Couldn't find pcl directory path which is {}, please check the path".format(self.pcd_dir_path))
            return

        for file in [f for f in os.listdir(self.ply_dir_path) if os.path.isfile(os.path.join(self.ply_dir_path, f))]:
            if ".ply" in file or ".PLY" in file:
                self.ply_file_name = file
                ply_file_path = os.path.join(self.ply_dir_path, self.ply_file_name)
                break
            else:
                ply_file_path = None
            
        if ply_file_path == None:
            self.get_logger().error("Couldn't find ply file in {}, please check your path!".format(self.ply_dir_path))
            return
        
        shutil.move(ply_file_path, self.pcd_dir_path)
        pcd_file_name = self.ply_file_name.replace(".ply", ".pcd")
        ply_file_path = os.path.join(self.pcd_dir_path, self.ply_file_name)
        pcd_file_path = os.path.join(self.pcd_dir_path, pcd_file_name)
        
        point_cloud = open3d.io.read_point_cloud(ply_file_path)
        
        if not point_cloud.has_points():
            self.get_logger().error("Point Cloud Data is not in file: {}".format(ply_file_path))
            return
        
        open3d.io.write_point_cloud(pcd_file_path, point_cloud)
        
        self.get_logger().info("Convert {} to {}".format(self.ply_file_name, pcd_file_name))
        
        if self.auto_clean_tmp:
            for dir in [d for d in os.listdir(self.pcd_dir_path) if os.path.isdir(os.path.join(self.pcd_dir_path, d))]:
                if "tmp" in dir :
                    shutil.rmtree(os.path.join(self.pcd_dir_path, dir))
                    self.get_logger().info("Temporary Data is cleaned automatically!")
        
        # self.destroy_node()
        # return


def main():
    rclpy.init(args=sys.argv)
    
    node = Ply2Pcd()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
