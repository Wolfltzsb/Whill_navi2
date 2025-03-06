import open3d, os, yaml, numpy as np, rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from whill_navi2.modules.ros2_launch_utils import DataPath
from whill_navi2.modules.waypoint_pkg_utils import read_waypoints, Waypoint, write_waypoints

class ConvertWaypointNode(Node):
    def __init__(self):
        super().__init__("convert_waypoint_node")
        
        self.input_pcd_path = self.declare_parameter("input_pcd_path", os.path.join(DataPath().pcd_map_dir_path, "glim.pcd")).get_parameter_value().string_value
        self.target_waypoint_path = self.declare_parameter("target_waypoint_path", os.path.join(DataPath().waypoint_dir_path, "gnss_odometry_waypoint.yaml")).get_parameter_value().string_value
        self.source_waypoint_path = self.declare_parameter("source_waypoint_path", os.path.join(DataPath().waypoint_dir_path, "hdl_odometry_waypoint.yaml")).get_parameter_value().string_value

def main():
    rclpy.init()
    node = ConvertWaypointNode()
    
    # 示例数据：GNSS路径和点云路径 (x, y) 坐标
    pos_xy = []
    path = []
    target_waypoints = read_waypoints(node.target_waypoint_path)
    for target_waypoint in target_waypoints:
        pos_xy = [float(target_waypoint.pos_x), float(target_waypoint.pos_y)]
        path.append(pos_xy)
    
    target_path = np.array(path)  # GNSS路径
    # print(target_path)
    pos_xy.clear()
    path.clear()
    source_waypoints = read_waypoints(node.source_waypoint_path)
    for source_waypoint in source_waypoints:
        pos_xy = [float(source_waypoint.pos_x), float(source_waypoint.pos_y)]
        path.append(pos_xy)

    source_path = np.array(path)  # 点云路径
    # print(source_path)
    
    # 定义目标函数，计算距离
    def objective(params, source_path, target_path):
        # 提取旋转角度和位移
        theta = params[0]
        tx = params[1]
        ty = params[2]
        
        # 创建旋转矩阵
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])
        
        # 旋转并平移点云路径
        transformed_path = np.dot(source_path, rotation_matrix.T) + np.array([tx, ty])
        
        # 计算点云路径与GNSS路径的距离
        distances = np.sqrt(np.sum((transformed_path[:, np.newaxis, :] - target_path[np.newaxis, :, :])**2, axis=2))
        
        # 取每个点到最近点的距离
        min_distances = np.min(distances, axis=1)
        
        return min_distances

    # 初始化参数：旋转角度（0）、x位移（0）、y位移（0）
    initial_params = np.array([0, 0, 0])

    # 使用最小二乘法优化
    result = least_squares(objective, initial_params, args=(source_path, target_path))

    # 提取优化后的参数
    optimized_theta = result.x[0]
    optimized_tx = result.x[1]
    optimized_ty = result.x[2]

    # 应用优化后的旋转和平移
    rotation_matrix = np.array([[np.cos(optimized_theta), -np.sin(optimized_theta)],
                                [np.sin(optimized_theta), np.cos(optimized_theta)]])
    transformed_path = np.dot(source_path, rotation_matrix.T) + np.array([optimized_tx, optimized_ty])
    # print(transformed_path)

    # # 计算平移量，以使初始点对齐
    # initial_offset = target_path[0] - transformed_path[0]

    # # 对对齐后的点云路径应用平移量
    # transformed_path += initial_offset
    # print(transformed_path[0])
    
    output_waypoints = []
    for index in range(len(transformed_path)):
        waypoint = Waypoint()
        waypoint.pos_x = float(transformed_path[index][0])
        waypoint.pos_y = float(transformed_path[index][1])
        waypoint.pos_z = source_waypoints[index].pos_z
        rpy = euler_from_quaternion([
            source_waypoints[index].quat_x,
            source_waypoints[index].quat_y,
            source_waypoints[index].quat_z,
            source_waypoints[index].quat_w
        ])
        yaw = rpy[2] + optimized_theta
        quat = quaternion_from_euler(rpy[0], rpy[1], yaw)
        waypoint.quat_x = float(quat[0])
        waypoint.quat_y = float(quat[1])
        quat = quaternion_from_euler(rpy[0], rpy[1], yaw)
        waypoint.quat_x = float(quat[0])
        waypoint.quat_y = float(quat[1])
        waypoint.quat_z = float(quat[2])
        waypoint.quat_w = float(quat[3])
        waypoint.roll = float(rpy[0])
        waypoint.pitch = float(rpy[1])
        waypoint.yaw = float(yaw)
        waypoint.mode = int(source_waypoints[index].mode)
        waypoint.latitude = source_waypoints[index].latitude
        waypoint.longitude = source_waypoints[index].longitude
        waypoint.time_stamp = source_waypoints[index].time_stamp
        covariance = []
        for elem in source_waypoints[index].covariance:
            covariance.append(float(elem))
        waypoint.covariance = covariance
        output_waypoints.append(waypoint)
    write_waypoints(os.path.join(os.path.dirname(node.source_waypoint_path), "converted_" + os.path.basename(node.source_waypoint_path)), output_waypoints)
    
    output_waypoints.clear()
    waypoint = Waypoint()
    waypoint.pos_x = float(optimized_tx)
    waypoint.pos_y = float(optimized_ty)
    quat = quaternion_from_euler(0.0, 0.0, optimized_theta)
    waypoint.quat_x = float(quat[0])
    waypoint.quat_y = float(quat[1])
    waypoint.quat_z = float(quat[2])
    waypoint.quat_w = float(quat[3])
    waypoint.yaw = float(optimized_theta)
    write_waypoints(os.path.join(os.path.dirname(node.source_waypoint_path), "T_source_target_" + os.path.basename(node.source_waypoint_path)), [waypoint])

    if ".pcd" in node.input_pcd_path:
        pcd = open3d.io.read_point_cloud(node.input_pcd_path)
        points = np.asarray(pcd.points)

        # 生成旋转矩阵
        rotation_matrix = np.array([
            [np.cos(optimized_theta), -np.sin(optimized_theta), 0],
            [np.sin(optimized_theta), np.cos(optimized_theta), 0],
            [0, 0, 1]
        ])
        
        translation_vector = np.array([optimized_tx, optimized_ty, 0.0])

        rotated_points = points @ rotation_matrix.T
        
        # final_points = rotated_points + translation_vector
        final_points = rotated_points

        point_cloud = open3d.geometry.PointCloud()
        point_cloud.points = open3d.utility.Vector3dVector(final_points)
        
        open3d.io.write_point_cloud(os.path.join(os.path.dirname(node.input_pcd_path), "converted_" + os.path.basename(node.input_pcd_path)), point_cloud)
    
    # 可视化比较
    # 获取x和y轴的最大值和最小值
    x_max = max(np.max(target_path[:, 0]), np.max(transformed_path[:, 0]))
    y_max = max(np.max(target_path[:, 1]), np.max(transformed_path[:, 1]))
    x_min = min(np.min(target_path[:, 0]), np.min(transformed_path[:, 0]))
    y_min = min(np.min(target_path[:, 1]), np.min(transformed_path[:, 1]))
    
    x_max = x_max + abs(x_max / 10)
    x_min = x_min - abs(x_min / 10)
    y_max = y_max + abs(y_max / 10)
    y_min = y_min - abs(y_min / 10)

    # 设置窗口大小 (宽度和高度可以根据需求调整)
    plt.figure(figsize=(8, 8))

    # 绘制路径
    plt.plot(target_path[:, 0], target_path[:, 1], label='Target Path', color='cyan')
    plt.plot(transformed_path[:, 0], transformed_path[:, 1], label='Transformed Source Path', color='brown')

    # 设置x轴和y轴的范围 (使用最大和最小值)
    plt.xlim([x_min, x_max])
    plt.ylim([y_min, y_max])

    # 设置x轴和y轴比例相同
    plt.gca().set_aspect('equal', adjustable='box')

    # 设置标签和标题
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.title('Alignment with Least Squares')

    # 显示图像
    plt.show()

if __name__ == "__main__":
    main()