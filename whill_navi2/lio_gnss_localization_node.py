import rclpy.duration
import rclpy.node
import rclpy.time
import rclpy, tf2_ros, tf_transformations, os, pymap3d, math, sys, numpy as np, csv
import tf2_geometry_msgs
from scipy.optimize import least_squares
from whill_navi2.modules.waypoint_pkg_utils import read_waypoints, Waypoint
from whill_navi2.modules.ros2_launch_utils import DataPath
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from nmea_msgs.msg import Sentence
from robot_localization.srv import SetPose
from sklearn.neighbors import NearestNeighbors
import ruptures
from scipy.signal import savgol_filter
from scipy.stats import mannwhitneyu
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
import pymannkendall
from whill_navi2.modules.ros2_launch_utils import DataPath


def normalize_yaw(yaw):
    return (yaw + 2 * math.pi) % (2 * math.pi)

# def objective(params, target_path, source_path):
#     tx = params[0]
#     ty = params[1]
#     theta = params[2]
    
#     rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
#                                 [np.sin(theta), np.cos(theta)]])
#     transformed_path = np.dot(source_path, rotation_matrix.T) + np.array([tx, ty])
#     distances = np.sqrt(np.sum((transformed_path[:, np.newaxis, :] - target_path[np.newaxis, :, :])**2, axis=2))
#     min_distances = np.min(distances, axis=1)
#     return min_distances

def objective(params, target_tree, source_path):
    tx = params[0]
    ty = params[1]
    theta = params[2]
    
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                [np.sin(theta), np.cos(theta)]])
    transformed_path = np.dot(source_path, rotation_matrix.T) + np.array([tx, ty])
    distances, _ = target_tree.query(transformed_path, k=1)
    return distances

def detect_significant_changes(data, window_size=5, p_threshold=0.01):
    """
    检测显著跳变的拐点（结合平滑和统计检验）
    
    参数:
        data: 输入数据（1D数组）
        window_size: 平滑窗口大小（建议奇数，如5）
        p_threshold: Mann-Whitney U检验的显著性阈值
    
    返回:
        significant_changes: 显著跳变点的索引列表（0-based）
    """
    # 1. 数据平滑（Savitzky-Golay滤波器，保留趋势）
    smoothed = savgol_filter(data, window_length=window_size, polyorder=2)
    
    # 2. 拐点检测（PELT算法，适合快速检测）
    algo = ruptures.Pelt(model="rbf", min_size=window_size).fit(smoothed)
    change_points = algo.predict(pen=2)  # pen值越小，灵敏度越高
    
    # 3. 显著性验证（Mann-Whitney U检验）
    significant_changes = []
    for cp in change_points:
        if cp <= window_size or cp >= len(data) - window_size:
            continue  # 忽略边缘点
        
        # 提取拐点前后的数据段
        before = data[cp - window_size : cp]
        after = data[cp : cp + window_size]
        
        # 非参数检验（适合小样本）
        stat, p = mannwhitneyu(before, after, alternative='two-sided')
        if p < p_threshold:
            significant_changes.append(cp - 1)  # 转换为0-based索引
    
    return significant_changes

def compute_transform(gnss_points, lio_points, optimize=True):
    """
    计算GNSS与LIO坐标系间的变换
    输入:
        gnss_points: N×3数组，全局坐标系点
        lio_points: N×3数组，局部坐标系点
        optimize: 是否启用非线性优化
    输出:
        translation: 平移向量 [x, y, z]
        quaternion: 旋转四元数 [x, y, z, w]
        scale: 尺度因子
        rmse: 均方根误差
    """
    # SVD求解初始变换
    R, t, s = svd_umeyama(lio_points, gnss_points)
    
    # 非线性优化
    if optimize:
        R, t, s = nonlinear_refinement(R, t, s, lio_points, gnss_points)
    
    # 转换为四元数
    quaternion = Rotation.from_matrix(R).as_quat()
    
    # 计算误差
    transformed = s * (R @ lio_points.T).T + t
    errors = np.linalg.norm(gnss_points - transformed, axis=1)
    rmse = np.sqrt(np.mean(errors**2))
    
    return t, quaternion, s, rmse

def svd_umeyama(src, dst):
    """SVD求解旋转、平移、尺度"""
    src_centroid = np.mean(src, axis=0)
    dst_centroid = np.mean(dst, axis=0)
    src_centered = src - src_centroid
    dst_centered = dst - dst_centroid
    
    # 计算协方差矩阵
    H = src_centered.T @ dst_centered
    
    # SVD分解
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # 处理反射
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # 计算尺度
    scale = np.trace(np.diag(S)) / np.sum(src_centered**2)
    
    # 计算平移
    t = dst_centroid - scale * R @ src_centroid
    
    return R, t, scale

def nonlinear_refinement(R_init, t_init, s_init, src, dst):
    """非线性优化"""
    def cost(params):
        angle_axis = params[:3]
        t = params[3:6]
        s = params[6]
        R = Rotation.from_rotvec(angle_axis).as_matrix()
        pred = s * (R @ src.T).T + t
        return (dst - pred).flatten()
    
    # 初始参数：旋转向量+平移+尺度
    x0 = np.concatenate([
        Rotation.from_matrix(R_init).as_rotvec(),
        t_init,
        [s_init]
    ])
    
    res = least_squares(cost, x0, method='lm', max_nfev=200)
    
    R_opt = Rotation.from_rotvec(res.x[:3]).as_matrix()
    t_opt = res.x[3:6]
    s_opt = res.x[6]
    
    return R_opt, t_opt, s_opt

class LioGnssLocalizationNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("lio_gnss_localization_node")
        
        T_utm_topomap_file = self.declare_parameter(name="T_utm_topomap_file", value=os.path.join(DataPath().waypoint_base_path, "GeoPoints", "T_utm_topomap_topological_map.yaml")).get_parameter_value().string_value
        T_topomap_liomap_file = self.declare_parameter(name="T_topomap_liomap_file", value=os.path.join(DataPath().waypoint_base_path, "GeoPoints", "T_topomap_liomap_topological_map.yaml")).get_parameter_value().string_value
        self.map_frame = self.declare_parameter(name="map_frame", value="lio_map").get_parameter_value().string_value
        self.buffer_len = self.declare_parameter(name="buffer_len", value=60).get_parameter_value().integer_value
        self.high_elevation_num_threshold = self.declare_parameter(name="high_elevation_num_threshold", value=12).get_parameter_value().integer_value
        self.translation_tollerance = self.declare_parameter(name="translation_tollerance", value=1.0).get_parameter_value().double_value
        self.rotation_threshold = self.declare_parameter(name="rotation_threshold", value=0.02).get_parameter_value().double_value
        self.front_speed = self.declare_parameter(name="front_speed", value=0.5).get_parameter_value().double_value
        self.turn_speed = self.declare_parameter(name="turn_speed", value=0.5).get_parameter_value().double_value
        self.snr_threshold = self.declare_parameter(name="snr_threshold", value=20.0).get_parameter_value().double_value
        self.oi_threshold = self.declare_parameter(name="oi_threshold", value=10.0).get_parameter_value().double_value
        self.hdop_threshold = self.declare_parameter(name="hdop_threshold", value=1.0).get_parameter_value().double_value
        
        self.tf2_static_broadcaster = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster(self)
        self.T_utm_topomap_file_wayoint = read_waypoints(T_utm_topomap_file)[0]
        self.T_topomap_liomap_file_wayoint = read_waypoints(T_topomap_liomap_file)[0]
        # self.topomap_waypoints = read_waypoints(os.path.join(os.path.dirname(T_topomap_liomap_file), "topological_map.yaml"))
        
        # self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))
        
        self.transforms = {}
        transform = tf2_ros.transform_broadcaster.TransformStamped()
        transform.header.frame_id = "utm"
        transform.child_frame_id = "topomap"
        transform.transform.translation.x = self.T_utm_topomap_file_wayoint.pos_x
        transform.transform.translation.y = self.T_utm_topomap_file_wayoint.pos_y
        self.get_logger().info("T_utm_topomap: [x: {}, y: {}]".format(self.T_utm_topomap_file_wayoint.pos_x, self.T_utm_topomap_file_wayoint.pos_y))
        # self.tf2_static_broadcaster.sendTransform(transform=transform)
        self.transforms["T_utm_topomap"] = transform
        
        transform = tf2_ros.transform_broadcaster.TransformStamped()
        transform.header.frame_id = "topomap"
        transform.child_frame_id = self.map_frame
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, self.T_topomap_liomap_file_wayoint.yaw)
        # q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, 0.42)
        transform.transform.rotation.x = q_x
        transform.transform.rotation.y = q_y
        transform.transform.rotation.z = q_z
        transform.transform.rotation.w = q_w
        # transform.transform.rotation.x = self.T_topomap_liomap_file_wayoint.quat_x
        # transform.transform.rotation.y = self.T_topomap_liomap_file_wayoint.quat_y
        # transform.transform.rotation.z = self.T_topomap_liomap_file_wayoint.quat_z
        # transform.transform.rotation.w = self.T_topomap_liomap_file_wayoint.quat_w
        self.get_logger().info("T_topomap_liomap: [yaw: {}]".format(self.T_topomap_liomap_file_wayoint.yaw))
        self.transforms["T_topomap_liomap"] = transform
        self.tf2_static_broadcaster.sendTransform(transform=[self.transforms["T_utm_topomap"], self.transforms["T_topomap_liomap"]])
        self.gnss_odometry_publisher = self.create_publisher(Odometry, "/gnss/odometry", 10)
        self.origin_odometry_publisher = self.create_publisher(Odometry, "/lio/origin/odometry", 10)
        self.fusing_odometry_publisher = self.create_publisher(Odometry, "/lio/fusing/odometry", 10)
        self.gnss_path_publisher = self.create_publisher(Path, "/gnss/path", 10)
        self.lio_path_publisher = self.create_publisher(Path, "/lio/path", 10)
        
        self.whill_odom_sub = self.create_subscription(Odometry, "/whill/odometry", self.on_whill_odom_callback, 10)
        self.nmea_sub = self.create_subscription(Sentence, "/ublox/nmea", self.on_nmea_callback, 60)
        self.set_pose_lio_client = self.create_client(SetPose, "/glim_rosnode/set_pose")
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)
        # self.listen_origin_lio_timer = self.create_timer(60 / self.buffer_len, self.on_lio_callback)
        # self.initial_direction_timer = self.create_timer(60 / self.buffer_len, self.update_transformation)
        
        self.initial_lio_buffer = []
        self.initial_gnss_buffer = []
        self.lio_buffer = []
        self.transformed_lio_buffer = []
        self.gnss_buffer = []
        # self.lio_buffer = []
        # self.origin_lio_buffer = []
        self.gsv_buffer = []
        self.gsv_eva_buffer = []
        self.nmea_recieved_time = 0.0
        self.nmea_gga_last_time = 0.0
        self.initial_rotation_yaw = 0.0
        self.gsv_index = 0
        self.is_indoor = False
        self.is_initialized = False
        self.moving_on = False
        gsv_index = 0
        transformation_index = 0
        odometry_index = 0
        for paths, dirs, names in os.walk(DataPath().waypoint_base_path + "/Backup"):
            for name in names:
                if "gsv" in name:
                    gsv_index += 1
                elif "transformation" in name:
                    transformation_index += 1
                elif "odometry" in name:
                    odometry_index += 1
        self.gsv_eva_csv_file_path = DataPath().waypoint_base_path + "/Backup" + "/gsv_evaluation_{}.csv".format(gsv_index)
        with open(self.gsv_eva_csv_file_path, "a") as f:
            csv.DictWriter(f, fieldnames="is_indoor, is_obstructed,num_satellites,num_high_elevation,avg_snr,weighted_snr,oi,hdop".split(",")).writeheader()
        self.gga_eva_csv_file_path = DataPath().waypoint_base_path + "/Backup" + "/odometry_evaluation_{}.csv".format(odometry_index)
        with open(self.gga_eva_csv_file_path, "a") as f:
            csv.DictWriter(f, fieldnames=["hdop", "gnss_x", "gnss_y", "transformed_x", "transformed_y", "delta_distance"]).writeheader()
        self.gga_eva_least_squares_csv_file_path = DataPath().waypoint_base_path + "/Backup" + "/transformation_by_least_squares_{}.csv".format(transformation_index)
        with open(self.gga_eva_least_squares_csv_file_path, "a") as f:
            csv.DictWriter(f, fieldnames=["hdop", "translation_x", "translation_y", "rotation_yaw", "scale", "rmse"]).writeheader()
            
    # def on_lio_callback(self):
    #     if not self.moving_on:
    #         return
        
    #     if self.tf_buffer.can_transform(self.map_frame, "base_link", time=rclpy.time.Time()):
    #         origin_lio_transform = self.tf_buffer.lookup_transform(self.map_frame, "base_link", time=rclpy.time.Time())
    #         self.origin_lio_buffer.append((origin_lio_transform.transform.translation.x, origin_lio_transform.transform.translation.y))
        
        # if self.is_initialized and len(self.origin_lio_buffer) > self.buffer_len - 1:
        #     source_path = []
        #     target_path = []
        #     for index in range(len(self.origin_lio_buffer)):
        #         source_path.append([self.origin_lio_buffer[index][0], self.origin_lio_buffer[index][1]])
        #     for waypoint in self.topomap_waypoints:
        #         target_path.append([waypoint.pos_x, waypoint.pos_y])
        #     source_path = np.array(source_path)
        #     target_path = np.array(target_path)
        #     initial_params = np.array([0.0, 0.0, 0.0])
            
        #     result = least_squares(objective, initial_params, args=(source_path, target_path), method="lm")
        #     if self.tf_buffer.can_transform("topomap", self.map_frame, time=rclpy.time.Time()):
        #         T_topomap_liomap = self.tf_buffer.lookup_transform("topomap", self.map_frame, time=rclpy.time.Time())
        #         # T_topomap_liomap.transform.translation.x = result.x[0]
        #         # T_topomap_liomap.transform.translation.y = result.x[1]
        #         q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, result.x[2])
        #         T_topomap_liomap.transform.rotation.x = q_x
        #         T_topomap_liomap.transform.rotation.y = q_y
        #         T_topomap_liomap.transform.rotation.z = q_z
        #         T_topomap_liomap.transform.rotation.w = q_w
        #         self.tf2_static_broadcaster.sendTransform(transform=T_topomap_liomap)
        #         self.origin_lio_buffer.clear()
        #         with open(self.gga_eva_least_squares_csv_file_path, "a") as f:
        #             csv.DictWriter(f, fieldnames=["gnss_x", "gnss_y", "transformed_x", "transformed_y", "delta_distance"]).writerow({"hdop": self.gnss_buffer[-1][3], "translation_x": T_topomap_liomap.transform.translation.x, "translation_y": T_topomap_liomap.transform.translation.y, "rotation_yaw": result.x[2]})
        #     else:
        #         pass
        
    def on_whill_odom_callback(self, msg : Odometry):
        if not self.moving_on and msg.twist.twist.linear.x > 0.1:
            self.moving_on = True
        if abs(self.get_clock().now().nanoseconds * 1e-9 - self.nmea_gga_last_time) < 0.1:
            self.front_speed = msg.twist.twist.linear.x
            self.turn_speed = msg.twist.twist.angular.z
            # print(self.front_speed, self.turn_speed)

    def update_transformation(self, target_path, source_path):
        # if len(self.origin_lio_buffer) > self.buffer_len - 1 and not self.is_initialized:
            # source_path = []
            # target_path = []
            # for index in range(len(self.origin_lio_buffer)):
            #     source_path.append([self.origin_lio_buffer[index][0], self.origin_lio_buffer[index][1]])
            # for waypoint in self.topomap_waypoints:
            #     target_path.append([waypoint.pos_x, waypoint.pos_y])
            if self.tf_buffer.can_transform("topomap", self.map_frame, time=rclpy.time.Time()):
                T_topomap_liomap = self.tf_buffer.lookup_transform("topomap", self.map_frame, time=rclpy.time.Time())
            source_path = np.array(source_path)
            target_path = np.array(target_path)
            initial_params_yaw = euler_from_quaternion([0.0, 0.0, T_topomap_liomap.transform.rotation.z, T_topomap_liomap.transform.rotation.w])[2]
            # initial_params = np.array([0.0, 0.0, 0.0])
            initial_params = np.array([T_topomap_liomap.transform.translation.x, T_topomap_liomap.transform.translation.y, initial_params_yaw])
            # result = least_squares(objective, initial_params, args=(target_path, source_path), method="lm")
            target_tree = cKDTree(target_path)
            result = least_squares(objective, initial_params, args=(target_tree, source_path), loss="huber", f_scale=1.0)
            p_x = result.x[0]
            p_y = result.x[1]
            rot_yaw = result.x[2]
            q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rot_yaw)
            z_col = np.zeros((source_path.shape[0], 1), dtype=source_path.dtype)
            source_path = np.hstack([source_path, z_col])
            z_col = np.zeros((target_path.shape[0], 1), dtype=target_path.dtype)
            target_path = np.hstack([target_path, z_col])
            translation, quaternion, scale, rmse = compute_transform(target_path, source_path)
            # p_x = translation[0]
            # p_y = translation[1]
            # q_x, q_y, q_z, q_w = quaternion
            # if not self.is_initialized:
            #     self.initial_rotation_yaw = tf_transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])[2]
            # if abs(self.initial_rotation_yaw - tf_transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])[2]) > 0.005:
            #     return
            # if not self.is_initialized:
            #     self.initial_rotation_yaw = rot_yaw
            # if abs(self.initial_rotation_yaw - rot_yaw) > self.rotation_threshold:
            #     return

            if self.tf_buffer.can_transform("topomap", self.map_frame, time=rclpy.time.Time()):
                # if not self.is_initialized:
                #     T_topomap_liomap.transform.translation.x = p_x
                #     T_topomap_liomap.transform.translation.y = p_y
                # if not self.is_initialized:
                T_topomap_liomap.transform.translation.x = p_x
                T_topomap_liomap.transform.translation.y = p_y
                T_topomap_liomap.transform.rotation.x = q_x
                T_topomap_liomap.transform.rotation.y = q_y
                T_topomap_liomap.transform.rotation.z = q_z
                T_topomap_liomap.transform.rotation.w = q_w
                self.tf2_static_broadcaster.sendTransform(transform=[self.transforms["T_utm_topomap"], T_topomap_liomap])
                with open(self.gga_eva_least_squares_csv_file_path, "a") as f:
                    if self.is_initialized:
                        if len(self.gnss_buffer) != 0:
                            csv.DictWriter(f, fieldnames=["hdop", "translation_x", "translation_y", "rotation_yaw", "scale", "rmse"]).writerow({"hdop": self.gnss_buffer[-1][3], "translation_x": T_topomap_liomap.transform.translation.x, "translation_y": T_topomap_liomap.transform.translation.y, "rotation_yaw": result.x[2], "scale": float(scale), "rmse": float(rmse)})
                            print({"hdop": self.gnss_buffer[-1][3], "translation_x": T_topomap_liomap.transform.translation.x, "translation_y": T_topomap_liomap.transform.translation.y, "rotation_yaw": result.x[2], "scale": float(scale), "rmse": float(rmse)})
                        else:
                            csv.DictWriter(f, fieldnames=["hdop", "translation_x", "translation_y", "rotation_yaw", "scale", "rmse"]).writerow({"hdop": self.initial_gnss_buffer[-1][3], "translation_x": T_topomap_liomap.transform.translation.x, "translation_y": T_topomap_liomap.transform.translation.y, "rotation_yaw": result.x[2], "scale": float(scale), "rmse": float(rmse)})
                            print({"hdop": self.initial_gnss_buffer[-1][3], "translation_x": T_topomap_liomap.transform.translation.x, "translation_y": T_topomap_liomap.transform.translation.y, "rotation_yaw": result.x[2], "scale": float(scale), "rmse": float(rmse)})
                    else:
                        csv.DictWriter(f, fieldnames=["hdop", "translation_x", "translation_y", "rotation_yaw", "scale", "rmse"]).writerow({"hdop": self.initial_gnss_buffer[-1][3], "translation_x": p_x, "translation_y": p_y, "rotation_yaw": result.x[2], "scale": float(scale), "rmse": float(rmse)})
                        print({"hdop": self.initial_gnss_buffer[-1][3], "translation_x": p_x, "translation_y": p_y, "rotation_yaw": result.x[2], "scale": float(scale), "rmse": float(rmse)})
    
    def on_nmea_callback(self, msg : Sentence):
        if not self.moving_on:
            return

        delta_dis_gnss = 0.0
        delta_dis_lio = 0.0
        delta_yaw_gnss = 0.0
        delta_yaw_lio = 0.0
        can_transform = self.tf_buffer.can_transform("topomap", "base_link", time=rclpy.time.Time())
        can_ld_transform = self.tf_buffer.can_transform(self.map_frame, "base_link", time=rclpy.time.Time())
        if can_transform and can_ld_transform:
            transform = self.tf_buffer.lookup_transform("topomap", "base_link", time=rclpy.time.Time())
            ld_transform = self.tf_buffer.lookup_transform(self.map_frame, "base_link", time=rclpy.time.Time())
        else:
            return
        
        if msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.nmea_recieved_time >= 0.1:
            self.nmea_recieved_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            satellites = self.parse_gsv(self.gsv_buffer)
            gsv_evaluation_dict = self.analyze_visibility(satellites)
            if len(self.gsv_eva_buffer) < self.buffer_len * 2:
                self.gsv_eva_buffer.append(gsv_evaluation_dict)
            else:
                self.gsv_eva_buffer.pop(0)
                self.gsv_eva_buffer.append(gsv_evaluation_dict)
            
            # high_elevation_datas = []
            # avg_snr_datas = []
            # for index in range(len(self.gsv_eva_buffer)):
            #     high_elevation_datas.append(self.gsv_eva_buffer[index]["num_high_elevation"])
            #     avg_snr_datas.append(self.gsv_eva_buffer[index]["avg_snr"])
            
            # window_size = int(len(self.gsv_eva_buffer) / 5)
            # high_elevation_datas = np.array(high_elevation_datas)
            # avg_snr_datas = np.array(avg_snr_datas)
            # if window_size and self.is_initialized:
            if self.is_initialized:
                # algo = ruptures.Pelt(model="rbf", min_size=window_size).fit(high_elevation_datas)
                # high_elevation_changes = algo.predict(pen=window_size / 2)
                # algo = ruptures.Pelt(model="rbf", min_size=window_size).fit(avg_snr_datas)
                # avg_snr_changes = algo.predict(pen=window_size / 2)
                # high_elev_trend = pymannkendall.original_test(high_elevation_datas[- window_size : ], alpha=0.05)
                # snr_trend = pymannkendall.original_test(avg_snr_datas[- window_size : ], alpha=0.05)
            
                # high_elevation_ok = False
                # avg_snr_ok = False
                # trend_ok = False
                
                # if high_elevation_changes and avg_snr_changes:
                #     high_elevation_changes.sort()
                #     avg_snr_changes.sort()
                #     if high_elevation_changes[-1] / len(self.gsv_eva_buffer) < 0.5:
                #         high_elevation_ok = True
                #     if avg_snr_changes[-1] / len(self.gsv_eva_buffer) < 0.5:
                #         avg_snr_ok = True
                
                # if "no trend" in high_elev_trend.trend or "no trend" in snr_trend.trend:
                #     trend_ok = True
                if not gsv_evaluation_dict["is_obstructed"]:
                    if gsv_evaluation_dict["num_high_elevation"] >= self.high_elevation_num_before or gsv_evaluation_dict["avg_snr"] >= self.high_elevation_num_before:
                    # if (high_elevation_ok or avg_snr_ok or trend_ok) and not gsv_evaluation_dict["is_obstructed"]:
                        self.is_indoor = False
                        self.high_elevation_num_before = 0
                        self.avg_snr_before = 0.0
                else:
                    self.is_indoor = True
                    # if len(self.gnss_buffer) >= self.buffer_len + 3:
                    #     # source_path = []
                    #     # target_path = []
                    #     # source_path.append([0.0, 0.0])
                    #     # target_path.append([0.0, 0.0])
                    #     # for index in range(len(self.initial_gnss_buffer)):
                    #     #     if self.initial_gnss_buffer[index][3] < self.hdop_threshold or not self.initial_gnss_buffer[index][4]:
                    #     #         source_path.append([self.initial_gnss_buffer[index][0], self.initial_gnss_buffer[index][1]])
                    #     #         target_path.append([self.initial_lio_buffer[index][0], self.initial_lio_buffer[index][1]])
                    #     # source_path = [[initial_lio[0], initial_lio[1]] for initial_lio in self.initial_lio_buffer]
                    #     # source_path.extend([[lio[0], lio[1]] for lio in self.lio_buffer[:-3]])
                    #     # target_path = [[initial_gnss[0], initial_gnss[1]] for initial_gnss in self.initial_gnss_buffer]
                    #     # target_path.extend([[gnss[0], gnss[1]] for gnss in self.gnss_buffer[:-3]])
                    #     if self.tf_buffer.can_transform("topomap", self.map_frame, time=rclpy.time.Time()):
                    #         # self.update_transformation(target_path=target_path, source_path=source_path)
                    #         T_topomap_lio_map = self.tf_buffer.lookup_transform("topomap", self.map_frame, time=rclpy.time.Time())
                    #         T_topomap_lio_map.transform.translation.x = self.gnss_buffer[-3][0] - self.transformed_lio_buffer[-3][0]
                    #         T_topomap_lio_map.transform.translation.y = self.gnss_buffer[-3][1] - self.transformed_lio_buffer[-3][1]
                    #         self.tf2_static_broadcaster.sendTransform(transform=[self.transforms["T_utm_topomap"], T_topomap_lio_map])                       
                    self.gsv_eva_buffer.pop()
                    if len(self.gsv_eva_buffer) > 1:
                        self.high_elevation_num_before = int(np.array([sat["num_high_elevation"] for sat in self.gsv_eva_buffer]).mean())
                        self.avg_snr_before = float(np.array([sat["avg_snr"] for sat in self.gsv_eva_buffer]).mean())

                
                # print("high_elevation_ok: {}".format(high_elevation_ok), "avg_snr_ok: {}".format(avg_snr_ok), "is_obstructed: {}".format(gsv_evaluation_dict["is_obstructed"]))
            else:
                if not gsv_evaluation_dict["is_obstructed"]:
                    self.is_indoor = False
                    self.high_elevation_num_before = 0
                    self.avg_snr_before = 0.0
                else:
                    self.is_indoor = True
                    self.gsv_eva_buffer.pop()
                    self.high_elevation_num_before = gsv_evaluation_dict["num_high_elevation"]
                    self.avg_snr_before = gsv_evaluation_dict["avg_snr"]

            
            gsv_evaluation_dict["is_indoor"] = self.is_indoor
            gsv_evaluation_dict["num_satellites"] = len(satellites)
            if not self.is_initialized:
                if self.initial_gnss_buffer:
                    gsv_evaluation_dict["hdop"] = self.initial_gnss_buffer[-1][3]
            else:
                if self.gnss_buffer:
                    gsv_evaluation_dict["hdop"] = self.initial_gnss_buffer[-1][3]
            with open(self.gsv_eva_csv_file_path, "a") as f:
                csv.DictWriter(f, fieldnames="is_indoor,is_obstructed,num_satellites,num_high_elevation,avg_snr,weighted_snr,oi,hdop".split(",")).writerow(gsv_evaluation_dict)
            self.gsv_buffer.clear()
        elif msg.sentence.startswith("$GPGSV") or msg.sentence.startswith("$GLGSV") or msg.sentence.startswith("$GBGSV") or msg.sentence.startswith("$GAGSV"):
            self.gsv_buffer.append(msg.sentence)
            # print(msg.sentence)
        elif msg.sentence.startswith("$GPGGA") or msg.sentence.startswith("$GNGGA"):
            nmea_gga = self.parse_gga(msg.sentence)
            if nmea_gga:
                x, y, h = pymap3d.enu.geodetic2enu(nmea_gga["latitude"], nmea_gga["longitude"], 0.0, self.T_utm_topomap_file_wayoint.latitude, self.T_utm_topomap_file_wayoint.longitude, 0.0)
                x_lio = ld_transform.transform.translation.x
                y_lio = ld_transform.transform.translation.y
                yaw_lio = tf_transformations.euler_from_quaternion([ld_transform.transform.rotation.x, ld_transform.transform.rotation.y, ld_transform.transform.rotation.z, ld_transform.transform.rotation.w])[2]
                transformed_x_lio = transform.transform.translation.x
                transformed_y_lio = transform.transform.translation.y
                transformed_yaw_lio = tf_transformations.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])[2]
                
                stamp = self.get_clock().now().to_msg()
                odom = Odometry()
                odom.header.frame_id = "topomap"
                odom.header.stamp = stamp
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.position.z = 0.0
                if len(self.gnss_buffer):
                    tmp_qx, tmp_qy, tmp_qz, tmp_qw = quaternion_from_euler(0.0, 0.0, math.pi - normalize_yaw(self.gnss_buffer[-1][2]))
                    odom.pose.pose.orientation.x = tmp_qx
                    odom.pose.pose.orientation.y = tmp_qy
                    odom.pose.pose.orientation.z = tmp_qz
                    odom.pose.pose.orientation.w = tmp_qw
                elif len(self.initial_gnss_buffer):
                    tmp_qx, tmp_qy, tmp_qz, tmp_qw = quaternion_from_euler(0.0, 0.0, math.pi - normalize_yaw(self.initial_lio_buffer[-1][2]))
                    odom.pose.pose.orientation.x = tmp_qx
                    odom.pose.pose.orientation.y = tmp_qy
                    odom.pose.pose.orientation.z = tmp_qz
                    odom.pose.pose.orientation.w = tmp_qw
                self.gnss_odometry_publisher.publish(odom)
                odom = Odometry()
                odom.header.frame_id = ld_transform.header.frame_id
                odom.header.stamp = stamp
                odom.child_frame_id = ld_transform.child_frame_id
                odom.pose.pose.position.x = x_lio
                odom.pose.pose.position.y = y_lio
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation.x = ld_transform.transform.rotation.x
                odom.pose.pose.orientation.y = ld_transform.transform.rotation.y
                odom.pose.pose.orientation.z = ld_transform.transform.rotation.z
                odom.pose.pose.orientation.w = ld_transform.transform.rotation.w
                self.origin_odometry_publisher.publish(odom)
                odom = Odometry()
                odom.header.frame_id = transform.header.frame_id
                odom.header.stamp = stamp
                odom.child_frame_id = transform.child_frame_id
                odom.pose.pose.position.x = transformed_x_lio
                odom.pose.pose.position.y = transformed_y_lio
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation.x = transform.transform.rotation.x
                odom.pose.pose.orientation.y = transform.transform.rotation.y
                odom.pose.pose.orientation.z = transform.transform.rotation.z
                odom.pose.pose.orientation.w = transform.transform.rotation.w
                self.fusing_odometry_publisher.publish(odom)
                gnss_path = Path()
                lio_path = Path()
                gnss_path.header.frame_id = "topomap"
                gnss_path.header.stamp = self.get_clock().now().to_msg()
                lio_path.header.frame_id = "topomap"
                lio_path.header.stamp = self.get_clock().now().to_msg()
                if self.is_initialized:
                    for index in range(len(self.gnss_buffer)):
                        pose = PoseStamped()
                        pose.pose.position.x = self.gnss_buffer[index][0]
                        pose.pose.position.y = self.gnss_buffer[index][1]
                        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, self.gnss_buffer[index][2])
                        pose.pose.orientation.x = q_x
                        pose.pose.orientation.y = q_y
                        pose.pose.orientation.z = q_z
                        pose.pose.orientation.w = q_w
                        gnss_path.poses.append(pose)
                        
                        pose.pose.position.x = self.transformed_lio_buffer[index][0]
                        pose.pose.position.y = self.transformed_lio_buffer[index][1]
                        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, self.transformed_lio_buffer[index][2])
                        pose.pose.orientation.x = q_x
                        pose.pose.orientation.y = q_y
                        pose.pose.orientation.z = q_z
                        pose.pose.orientation.w = q_w
                        lio_path.poses.append(pose)
                    self.gnss_path_publisher.publish(gnss_path)
                    self.lio_path_publisher.publish(lio_path)
                else:
                    for index in range(len(self.initial_gnss_buffer)):
                        pose = PoseStamped()
                        pose.pose.position.x = self.initial_gnss_buffer[index][0]
                        pose.pose.position.y = self.initial_gnss_buffer[index][1]
                        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, self.initial_gnss_buffer[index][2])
                        pose.pose.orientation.x = q_x
                        pose.pose.orientation.y = q_y
                        pose.pose.orientation.z = q_z
                        pose.pose.orientation.w = q_w
                        gnss_path.poses.append(pose)
                        
                        pose.pose.position.x = self.initial_lio_buffer[index][0]
                        pose.pose.position.y = self.initial_lio_buffer[index][1]
                        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, self.initial_lio_buffer[index][2])
                        pose.pose.orientation.x = q_x
                        pose.pose.orientation.y = q_y
                        pose.pose.orientation.z = q_z
                        pose.pose.orientation.w = q_w
                        lio_path.poses.append(pose)
                    # self.gnss_path_publisher.publish(gnss_path)
                    self.lio_path_publisher.publish(lio_path)
                
                if len(self.gnss_buffer) == 0:
                    yaw = self.T_topomap_liomap_file_wayoint.yaw
                else:
                    yaw = math.atan2(y - self.gnss_buffer[-1][1], x - self.gnss_buffer[-1][0])
                    delta_dis_gnss = math.hypot((self.gnss_buffer[-1][0] - x), (self.gnss_buffer[-1][1] - y))
                    delta_yaw_gnss = abs(normalize_yaw(yaw) - normalize_yaw(self.gnss_buffer[-1][2]))
                
                if len(self.lio_buffer) == 0:
                    pass
                else:
                    delta_dis_lio = math.hypot((x_lio - self.lio_buffer[-1][0]), (y_lio - self.lio_buffer[-1][1]))
                    delta_yaw_lio = abs(normalize_yaw(yaw_lio) - normalize_yaw(self.lio_buffer[-1][2]))
                
                if not self.is_initialized:
                    if len(self.initial_gnss_buffer) < self.buffer_len:
                        # self.gnss_buffer.append((x, y, yaw, nmea_gga["hdop"], self.is_indoor))
                        self.initial_gnss_buffer.append((x, y, yaw, nmea_gga["hdop"], self.is_indoor))
                        # self.lio_buffer.append((x_lio, y_lio, yaw_lio))
                        self.initial_lio_buffer.append((x_lio, y_lio, yaw_lio))
                    else:
                        source_path = []
                        target_path = []
                        for index in range(len(self.initial_gnss_buffer)):
                            target_path.append([self.initial_gnss_buffer[index][0], self.initial_gnss_buffer[index][1]])
                            source_path.append([self.initial_lio_buffer[index][0], self.initial_lio_buffer[index][1]])
                        self.update_transformation(target_path, source_path)
                        self.is_initialized = True
                else:
                    if len(self.gnss_buffer) < self.buffer_len * 1000:
                        self.gnss_buffer.append((x, y, yaw, nmea_gga["hdop"], self.is_indoor))
                        self.lio_buffer.append((x_lio, y_lio, yaw_lio))
                        self.transformed_lio_buffer.append((transformed_x_lio, transformed_y_lio, transformed_yaw_lio))
                    else:
                        self.gnss_buffer.pop(0)
                        self.gnss_buffer.append((x, y, yaw, nmea_gga["hdop"], self.is_indoor))
                        self.lio_buffer.pop(0)
                        self.lio_buffer.append((x_lio, y_lio, yaw_lio))
                        self.transformed_lio_buffer.pop(0)
                        self.transformed_lio_buffer.append((transformed_x_lio, transformed_y_lio, transformed_yaw_lio))
                    
                # if len(self.lio_buffer) < self.buffer_len:
                #     self.lio_buffer.append((x_lio, y_lio, yaw_lio))
                # else:
                #     self.lio_buffer.pop(0)
                #     self.lio_buffer.append((x_lio, y_lio, yaw_lio))
                
                T_topomap_lio_map = self.tf_buffer.lookup_transform("topomap", self.map_frame, time=rclpy.time.Time())
                T_topomap_lio_map_yaw = tf_transformations.euler_from_quaternion([T_topomap_lio_map.transform.rotation.x, T_topomap_lio_map.transform.rotation.y, T_topomap_lio_map.transform.rotation.z, T_topomap_lio_map.transform.rotation.w])[2]
                
                # if not self.is_initialized:
                #     gnss_vector = math.atan2(y, x)
                #     lio_vector = math.atan2(y_lio, x_lio)
                #     delta_dis = math.hypot((x_lio - x), (y_lio - y))
                #     if abs(gnss_vector - lio_vector) <= 0.001:
                #         self.is_initialized = True
                #     if delta_dis > self.translation_tollerance and abs(gnss_vector - lio_vector) > self.rotation_threshold:
                        
                #         T_topomap_lio_map.transform.translation.x = x - x_lio
                #         T_topomap_lio_map.transform.translation.y = y - y_lio
                #         roll, pitch, tmp_yaw = tf_transformations.euler_from_quaternion([T_topomap_lio_map.transform.rotation.x, T_topomap_lio_map.transform.rotation.y, T_topomap_lio_map.transform.rotation.z, T_topomap_lio_map.transform.rotation.w])
                #         q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, tmp_yaw + gnss_vector - lio_vector)
                #         T_topomap_lio_map.transform.rotation.x = q_x
                #         T_topomap_lio_map.transform.rotation.y = q_y
                #         T_topomap_lio_map.transform.rotation.z = q_z
                #         T_topomap_lio_map.transform.rotation.w = q_w
                #         if not self.is_indoor and self.is_good_rotation:
                #             self.tf2_static_broadcaster.sendTransform(transform=T_topomap_lio_map)
                
                # if self.is_initialized:
                # if len(covariances) >= gnss_data_buffer_len / 2:
                # if covariance <= self.bad_covariance:
                    # atan_vector_lio = float(math.atan2(y_lio, x_lio))
                    # atan_vector_gnss = float(math.atan2(y, x))
                    # if abs(atan_vector_gnss - atan_vector_lio) > self.rotation_threshold:
                    #     r, p, T_topomap_lio_map_yaw = euler_from_quaternion([
                    #         T_topomap_lio_map.transform.rotation.x,
                    #         T_topomap_lio_map.transform.rotation.y,
                    #         T_topomap_lio_map.transform.rotation.z,
                    #         T_topomap_lio_map.transform.rotation.w,
                    #     ])
                    #     T_topomap_lio_map_yaw = T_topomap_lio_map_yaw + atan_vector_gnss - atan_vector_lio
                    #     # print(T_topomap_lio_map_yaw * 180 / math.pi)
                    #     q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, T_topomap_lio_map_yaw)
                    #     T_topomap_lio_map.transform.rotation.x = q_x
                    #     T_topomap_lio_map.transform.rotation.y = q_y
                    #     T_topomap_lio_map.transform.rotation.z = q_z
                    #     T_topomap_lio_map.transform.rotation.w = q_w
                        
                    #     delta_time = self.get_clock().now().nanoseconds * 1e-9 - self.nmea_gga_last_time
                    #     gnss_front_speed = delta_dis_gnss / delta_time
                    #     gnss_turn_speed = delta_yaw_gnss / delta_time
                    #     # if gnss_turn_speed <= self.front_speed and delta_yaw_gnss <= self.turn_speed:
                    #     if gnss_front_speed <= self.front_speed and gnss_turn_speed <= self.turn_speed and not self.is_indoor and self.is_good_rotation:
                    #         self.tf2_static_broadcaster.sendTransform(transform=[self.transforms["T_utm_topomap"], T_topomap_lio_map])
                    #         # self.lio_buffer.clear()
                    #         # self.gnss_buffer.clear()
                    #         with open(self.gga_eva_csv_file_path, "a") as f:
                    #             csv.DictWriter(f, fieldnames=["gnss_x", "gnss_y", "transformed_x", "transformed_y", "delta_distance"]).writerow({"hdop": nmea_gga["hdop"], "translation_x": T_topomap_lio_map.transform.translation.x, "translation_y": T_topomap_lio_map.transform.translation.y, "rotation_yaw": T_topomap_lio_map_yaw})

                    # transform = self.tf_buffer.lookup_transform("topomap", "base_link", time=rclpy.time.Time())
                    # x_lio = transform.transform.translation.x
                    # y_lio = transform.transform.translation.y
                    # delta_dis = math.hypot((x_lio - x), (y_lio - y))
                    # print("delta_distance: {}".format(delta_dis), "delta_lio_gnss: {}".format(abs(delta_dis_gnss - delta_dis_lio)), "delta_yaw_lio_gnss: {}".format(abs(delta_yaw_gnss - delta_yaw_lio)))
                    # if delta_dis > self.translation_tollerance:
                    #     T_topomap_lio_map.transform.translation.x = x - x_lio
                    #     T_topomap_lio_map.transform.translation.y = y - y_lio
                    #     delta_time = self.get_clock().now().nanoseconds * 1e-9 - self.nmea_gga_last_time
                    #     gnss_front_speed = delta_dis_gnss / delta_time
                    #     gnss_turn_speed = delta_yaw_gnss / delta_time
                    #     lio_front_speed = delta_dis_lio / delta_time
                    #     lio_turn_speed = delta_yaw_lio / delta_time

                    #     if not self.is_indoor and nmea_gga["hdop"] < self.hodop_threshold:
                    #         self.tf2_static_broadcaster.sendTransform(transform=[self.transforms["T_utm_topomap"], T_topomap_lio_map])                        
                    # print(100 ** nmea_gga["hdop"])
                
                delta_distance = math.hypot(x - transformed_x_lio, y - transformed_y_lio)
                if self.is_initialized:
                    # print("delta_distance: {}".format(delta_distance), ",", " is_indoor: {}".format(self.is_indoor))
                    if not self.is_indoor and nmea_gga["hdop"] < self.hdop_threshold and delta_dis_gnss <= delta_dis_lio * 1.01 and delta_yaw_gnss <= delta_yaw_lio * 1.01:
                        
                        source_path = []
                        target_path = []
                        for index in range(len(self.initial_gnss_buffer)):
                            target_path.append([self.initial_gnss_buffer[index][0], self.initial_gnss_buffer[index][1]])
                            source_path.append([self.initial_lio_buffer[index][0], self.initial_lio_buffer[index][1]])
                        for index in range(len(self.gnss_buffer)):
                            target_path.append([self.gnss_buffer[index][0], self.gnss_buffer[index][1]])
                            source_path.append([self.lio_buffer[index][0], self.lio_buffer[index][1]])

                        self.update_transformation(target_path, source_path)

                        if self.tf_buffer.can_transform("topomap", "base_link", time=rclpy.time.Time()):
                            updated_transform = self.tf_buffer.lookup_transform("topomap", "base_link", time=rclpy.time.Time())
                            if math.hypot(x - updated_transform.transform.translation.x, y - updated_transform.transform.translation.y) > self.translation_tollerance:
                                T_topomap_lio_map.transform.translation.x += x - updated_transform.transform.translation.x
                                T_topomap_lio_map.transform.translation.y += y - updated_transform.transform.translation.y
                                self.tf2_static_broadcaster.sendTransform(transform=[self.transforms["T_utm_topomap"], T_topomap_lio_map])
                        # source_path = []
                        # target_path = []
                        # for index in range(len(self.initial_gnss_buffer)):
                        #     target_path.append([self.initial_gnss_buffer[index][0], self.initial_gnss_buffer[index][1]])
                        #     source_path.append([self.initial_lio_buffer[index][0], self.initial_lio_buffer[index][1]])
                        # for index in range(len(self.gnss_buffer)):
                        #     target_path.append([self.gnss_buffer[index][0], self.gnss_buffer[index][1]])
                        #     source_path.append([self.lio_buffer[index][0], self.lio_buffer[index][1]])

                        # self.update_transformation(target_path, source_path)
                
                if self.is_initialized:
                    for index in range(len(self.gnss_buffer)):
                        if self.gnss_buffer[index][3] > self.hdop_threshold or self.gnss_buffer[index][4]:
                            # origin_gnss_yaw = self.gnss_buffer[index][2]
                            # # (x, y, yaw, nmea_gga["hdop"], self.is_indoor)
                            # self.gnss_buffer[index] = (self.transformed_lio_buffer[index][0], self.transformed_lio_buffer[index][1], origin_gnss_yaw, 0.01, False)
                            self.gnss_buffer.pop(index)
                            self.lio_buffer.pop(index)
                            self.transformed_lio_buffer.pop(index)
                                                                
                with open(self.gga_eva_csv_file_path, "a") as f:
                    csv.DictWriter(f, fieldnames=["hdop", "gnss_x", "gnss_y", "transformed_x", "transformed_y", "delta_distance"]).writerow({"hdop": nmea_gga["hdop"], "gnss_x": x, "gnss_y": y, "transformed_x": transformed_x_lio, "transformed_y" : transformed_y_lio, "delta_distance" : delta_distance})
                    
                self.nmea_gga_last_time = self.get_clock().now().nanoseconds * 1e-9

    
    def parse_rmc(self, rmc_sentence):
        """
        Parse NMEA RMC sentence (GPRMC or GNRMC) and return parsed data.

        :param rmc_sentence: NMEA RMC sentence string
        :return: Dictionary with parsed data or None if parsing fails
        """
        if not rmc_sentence.startswith('$GPRMC') and not rmc_sentence.startswith('$GNRMC'):
            return None

        parts = rmc_sentence.split(',')
        if len(parts) < 12:
            return None

        try:
            time = parts[1]
            status = parts[2]
            latitude = float(parts[3])
            lat_dir = parts[4]
            longitude = float(parts[5])
            lon_dir = parts[6]
            speed = float(parts[7])
            course = float(parts[8])
            date = parts[9]

            # Validate directions
            if lat_dir not in ['N', 'S'] or lon_dir not in ['E', 'W']:
                return None

            # Convert latitude to decimal degrees
            lat_deg = int(latitude / 100)
            lat_min = latitude % 100
            latitude_dd = lat_deg + (lat_min / 60)
            if lat_dir == 'S':
                latitude_dd = -latitude_dd

            # Convert longitude to decimal degrees
            lon_deg = int(longitude / 100)
            lon_min = longitude % 100
            longitude_dd = lon_deg + (lon_min / 60)
            if lon_dir == 'W':
                longitude_dd = -longitude_dd

            # Convert date to YYYY-MM-DD
            day = date[0:2]
            month = date[2:4]
            year = date[4:6]
            year_str = '19' + year if int(year) < 50 else '20' + year
            date_full = f'{year_str}-{month}-{day}'

            # Create dictionary with parsed data
            rmc_data = {
                'time': time,
                'status': status,
                'latitude': latitude_dd,
                'longitude': longitude_dd,
                'speed_knots': speed,
                'course_true': course,
                'date': date_full
            }

            return rmc_data

        except (ValueError, IndexError):
            return None
    
    def parse_gsv(self, gsv_list):
        satellites = []
        for gsv in gsv_list:
            parts = gsv.split(',')
            if parts[0] in ['$GPGSV', '$GLGSV', '$GAGSV', '$GBGSV']:
                num_sats = int(parts[3])
                for i in range(0, min(num_sats, 4)):
                    index = 4 + i * 4
                    if index + 3 < len(parts):
                        prn = parts[index]
                        # print(parts[index + 1])
                        if parts[index + 1]:
                            elevation = float(parts[index + 1])
                            azimuth = float(parts[index + 2])
                        else:
                            elevation = 0.0
                            azimuth = 0.0
                        snr = parts[index + 3]
                        if snr:
                            satellites.append({
                                'system': parts[0][1:3],
                                'prn': prn,
                                'elevation': elevation,
                                'azimuth' : azimuth,
                                'snr': float(snr)
                            })
        return satellites
    
    def parse_gga(self, gga_sentence):
        if not gga_sentence.startswith(('$GPGGA', '$GNGGA')):
            return None

        parts = gga_sentence.split(',')

        if len(parts) < 15:
            return None

        try:
            time = parts[1]  # UTC时间，格式为hhmmss.ss
            latitude = float(parts[2])  # 纬度，格式为ddmm.mmmm
            lat_dir = parts[3]  # 纬度方向，N或S
            longitude = float(parts[4])  # 经度，格式为dddmm.mmmm
            lon_dir = parts[5]  # 经度方向，E或W
            fix_status = int(parts[6])  # 定位状态，0=无效，1=GPS定位，2=差分GPS定位
            num_sats = int(parts[7])  # 使用的卫星数量
            hdop = float(parts[8])  # 水平精度因子
            altitude = float(parts[9])  # 海拔高度，单位：米
            alt_unit = parts[10]  # 海拔高度单位，通常为'M'
            geoid_sep = float(parts[11])  # 大地水准面高度，单位：米
            geoid_unit = parts[12]  # 大地水准面高度单位，通常为'M'
            age_diff = parts[13]  # 差分GPS数据年龄，单位：秒
            diff_station = parts[14]  # 差分参考站ID

            # 将纬度和经度转换为十进制度
            latitude = (int(latitude / 100) + (latitude % 100) / 60) * (-1 if lat_dir == 'S' else 1)
            longitude = (int(longitude / 100) + (longitude % 100) / 60) * (-1 if lon_dir == 'W' else 1)

            # 返回解析后的数据
            return {
                'time': time,
                'latitude': latitude,
                'longitude': longitude,
                'fix_status': fix_status,
                'num_sats': num_sats,
                'hdop': hdop,
                'altitude': altitude,
                'geoid_sep': geoid_sep,
                'age_diff': age_diff,
                'diff_station': diff_station
            }
        except (ValueError, IndexError):
            # 如果解析过程中出现错误，返回None
            return None

    def analyze_visibility(self, satellites):
        weighted_snr_value = self.weighted_snr(satellites)
        high_elevation_sats = [sat for sat in satellites if sat['elevation'] > 45 and sat['snr'] >= self.snr_threshold]
        num_high_elevation = len(high_elevation_sats)
        oi = self.obstruction_index(satellites)
        is_obstructed = True
        
        snr_values = [sat['snr'] for sat in satellites]
        avg_snr = sum(snr_values) / len(snr_values) if snr_values else 0
        
        if num_high_elevation < self.high_elevation_num_threshold or avg_snr < self.snr_threshold:
            is_obstructed = True
        else:
            is_obstructed = False
        
        return {"is_obstructed": is_obstructed, "num_high_elevation": num_high_elevation, "avg_snr": avg_snr, "weighted_snr": weighted_snr_value, "oi": oi}
    
    def obstruction_index(self, satellites):
        high_elevation_sats = [sat for sat in satellites if sat['elevation'] > 45]
        num_high_elevation = len(high_elevation_sats)
        total_sats = len(satellites)
        avg_snr = sum(sat['snr'] for sat in satellites) / total_sats if total_sats else 0
        oi = (num_high_elevation / total_sats) * avg_snr if total_sats else 0
        return oi
    
    def weighted_snr(self, satellites):
        weighted_snr_sum = 0
        total_weight = 0
        for sat in satellites:
            elevation = sat['elevation']
            snr = sat['snr']
            weight = max(0.1, sat['elevation'] / 90)  # 权重与仰角相关，避免权重为0
            if elevation > 45:
                weight = weight * 2
            else:
                weight = weight * 0.5

            weighted_snr_sum += snr * weight
            total_weight += weight

        return weighted_snr_sum / total_weight if total_weight else 0
    
def main():
    rclpy.init(args=sys.argv)
    node = LioGnssLocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()
