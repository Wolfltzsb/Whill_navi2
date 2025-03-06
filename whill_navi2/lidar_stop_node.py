
import rclpy
from sensor_msgs_py.point_cloud2 import create_cloud, read_points, read_points_list
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Twist
from rclpy.duration import Duration
import open3d, numpy as np
import threading

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    # PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
]

class LidarStop(Node):
    def __init__(self):
        super().__init__("lidar_stop")
        
        self.point_in_sub = self.create_subscription(PointCloud2, "lumotive_ros/pointcloud", self.sub_callback, 10)
        # self.zero_velocity_pub = self.create_publisher(Twist, "whill/controller/cmd_vel", 100)
        self.stop_signal_pub = self.create_publisher(String, "~/stop", 2)
        self.stop_distance = self.declare_parameter("stop_distance", 0.8).get_parameter_value().double_value
        self.scan_min_range = self.declare_parameter("scan_min_range", - 45.0).get_parameter_value().double_value
        self.scan_max_range = self.declare_parameter("scan_max_range", 45.0).get_parameter_value().double_value
    
    def sub_callback(self, msg : PointCloud2):

        origin_points = list(read_points_list(msg, field_names=["x", "y", "z"], skip_nans=True))
        points = np.array(origin_points, dtype=np.float32)
        # 转换到球坐标系
        r = np.linalg.norm(points, axis=1)
        theta = np.arctan2(points[:, 1], points[:, 0])  # 水平角
        phi = np.arctan2(np.sqrt(points[:, 0]**2 + points[:, 1]**2), points[:, 2])  # 垂直角

        # 转为角度制
        theta_deg = np.degrees(theta)  # 水平角度

        # 过滤横向范围
        valid_mask = (theta_deg >= self.scan_min_range) & (theta_deg <= self.scan_max_range)
        filtered_points = points[valid_mask]
        filtered_r = r[valid_mask]
        filtered_theta_deg = theta_deg[valid_mask]
        # print(filtered_theta_deg)
        # 定义水平扫描的分段间隔（分辨率为 0.375°）
        horizontal_bins = np.arange(self.scan_min_range, self.scan_max_range + 0.375, 0.375)  # 每 0.375° 一个区间
        bin_indices = np.digitize(filtered_theta_deg, horizontal_bins)  # 按水平角度分配区间
        max_range = abs(self.scan_min_range - self.scan_max_range)

        # 初始化连续扫描目录
        continuous_index = 0
        # 遍历每个垂直扫描区间
        for i in range(1, len(horizontal_bins)):
            # 获取属于当前垂直扫描（bin）的点
            mask = bin_indices == i
            r_filtered = filtered_r[mask]  # 当前扫描中的 r 值
            # 判断 r < 制动距离的点数量
            count = np.sum(r_filtered < self.stop_distance)
            
            # 输出结果
            if count > 20:
                continuous_index += 1
                if continuous_index > 20:
                    # print("有障碍物")
                    send_data = String()
                    angle_now = i * 0.375 - max_range / 2.0
                    if angle_now >= 0.0:
                        send_data.data = "left/{}".format(angle_now)
                        self.get_logger().info(send_data.data)
                    else:
                        send_data.data = "right/{}".format(angle_now)
                        self.get_logger().info(send_data.data)
                    self.stop_signal_pub.publish(send_data)
                    # self.get_clock().sleep_for(Duration(seconds=5, nanoseconds=0))
                    return
                # print(f"水平扫描 {horizontal_bins[i-1]:.3f}° - {horizontal_bins[i]:.3f}° 中有 {count} 个点的 r 小于 {self.stop_distance} 米，超过 10 个。")
                # print(continuous_index)
            else:
                continuous_index = 0
                # print(f"水平扫描 {horizontal_bins[i-1]:.3f}° - {horizontal_bins[i]:.3f}° 中有 {count} 个点的 r 小于 {self.stop_distance} 米，不超过 10 个。")
        
        # if pcd_as_numpy_array.shape[1] != 3:
        #     raise ValueError("Point cloud data must have shape (N, 3)")

        # o3d_pcd = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(pcd_as_numpy_array))
        # downsampled_pcd = o3d_pcd.voxel_down_sample(voxel_size=self.voxel_resolution)
        # downsampled_pcd.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=2.0, max_nn=30))
        
        # z_axis = np.array([0, 0, 1])
        # normals = np.asarray(downsampled_pcd.normals)
        # indices_to_keep = []
        # for i, normal in enumerate(normals):
        #     dot_product = np.dot(normal, z_axis)
        #     if abs(dot_product) < 0.8:
        #         indices_to_keep.append(i)
        
        # pcd_filtered = downsampled_pcd.select_by_index(indices_to_keep)
        # points = np.asarray(pcd_filtered.points)
        # distances = np.linalg.norm(points, axis=1)
        
        # idx = 0     
        # for distance in distances:
        #     if distance <= self.stop_distance:
        #         idx = idx + 1
        
        # if idx / len(distances) >= 0.1:
        #     self.stop_flag = True
        #     self.get_logger().warn("Obstacle is there. wait for a moment...")
        #     # zero_speed = Twist()
        #     # zero_speed.linear.x = 0.0
        #     # zero_speed.angular.z = 0.0
            
        #     # self.get_logger().info("Obstacle nearby, STOP!!")
        #     # self.zero_velocity_pub.publish(zero_speed)

        # else:
        #     self.stop_flag = False
            

        # header = msg.header
        # header.stamp = self.get_clock().now().to_msg()
        # self.point_out_pub.publish(create_cloud(header, FIELDS, points))
        
def main():
    rclpy.init()
    node = LidarStop()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__" :
    main()
        
