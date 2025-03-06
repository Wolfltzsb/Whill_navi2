import rclpy
from rclpy.node import Node
from collections import deque
from sensor_msgs.msg import Imu, PointCloud2

class ImuPointCloudSync(Node):
    def __init__(self):
        super().__init__('imu_points_sync_node')

        # 创建 IMU 数据缓存
        self.imu_buffer = deque(maxlen=200)  # 200Hz, 0.1s 内最大 20 条数据

        # 订阅 IMU 数据（200Hz）
        self.imu_sub = self.create_subscription(
            Imu, '/adis/imu/data', self.imu_callback, 200)

        # 订阅点云数据（10Hz）
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.cloud_callback, 10)

        # 发布同步后的 IMU 数据
        self.imu_pub = self.create_publisher(Imu, '/adis/imu/data/synced', 10)
        # 发布原始点云数据
        self.cloud_pub = self.create_publisher(PointCloud2, '/velodyne_points/synced', 10)

    def imu_callback(self, msg):
        """IMU 回调函数，将数据存入缓冲区"""
        self.imu_buffer.append(msg)

    def cloud_callback(self, cloud_msg):
            """点云回调函数，发布前后 3 个 IMU 数据"""
            if not self.imu_buffer:
                self.get_logger().warn("IMU buffer is empty, skipping this frame!")
                return
            
            cloud_time = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9

            # 筛选 3 个早于点云的 IMU 数据
            prev_imu = [imu for imu in self.imu_buffer if imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9 < cloud_time]
            prev_imu = prev_imu[-10:] if len(prev_imu) >= 10 else prev_imu  # 取最后 3 个

            # 筛选 3 个晚于点云的 IMU 数据
            next_imu = [imu for imu in self.imu_buffer if imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9 > cloud_time]
            next_imu = next_imu[:10] if len(next_imu) >= 10 else next_imu  # 取前 3 个

            # 先发布 3 个早于点云的 IMU 数据
            for imu in prev_imu:
                self.imu_pub.publish(imu)

            # 发布点云数据
            self.cloud_pub.publish(cloud_msg)

            # 再发布 3 个晚于点云的 IMU 数据
            for imu in next_imu:
                self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPointCloudSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()