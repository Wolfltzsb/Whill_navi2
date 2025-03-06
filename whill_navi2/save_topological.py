import sys, os, yaml, rclpy, time, numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from std_msgs.msg import String
from whill_navi2.modules.waypoint_pkg_utils import (
    Waypoint, read_waypoints, write_waypoints
)
import pymap3d, math, tf_transformations, pyproj

def calculate_initial_yaw(data : list, num_points : int = 10) -> float:
    if len(data) < 2:
        raise ValueError("Point is not enough.")
    
    initial_points = data[:num_points]
    x_coords = np.array([point[0] for point in initial_points])
    y_coords = np.array([point[1] for point in initial_points])
    
    # 使用线性回归拟合直线
    coefficients = np.polyfit(x_coords, y_coords, 1)  # 一次多项式拟合
    slope = coefficients[0]  # 斜率
    
    # 计算yaw (方向角)
    initial_yaw = np.arctan2(y_coords[-1] - y_coords[0], x_coords[-1] - x_coords[0])
    
    return float(initial_yaw)


class GeoTopologicalMap(Node):
    
    def __init__(self):
        super().__init__("geo_topological_map")
        
        self.file_path = self.declare_parameter(
            name="file_path", 
            value=os.path.join(os.environ["HOME"], "Documents", "python_test.yaml")
        ).get_parameter_value().string_value
        self.polygon_sub = self.create_subscription(PolygonStamped, "topological_map", self.topo_sub_callback, 1)
        self.kml_path_sub = self.create_subscription(String, "kml_file_path", self.mv_kml_file, 1)
        self.file_number = 0
        
        self.get_logger().info("Ready to recieve topological information!!!")
    
    def topo_sub_callback(self, msg : PolygonStamped):
                        
        if msg.header.frame_id != "wgs84":
            self.get_logger().warn("The Frame ID is not wgs84. The GeoPoint is needed here!!!!")
            return

        key = 0
        # waypoints = {}
        waypoints = []
        initial_x = 0.0
        initial_y = 0.0
        for point in msg.polygon.points:
            # waypoints[key] = Waypoint()
            if key == 0:
                waypoint = Waypoint()
                waypoint.mode = Waypoint.WaypointMode.NORMAL
                waypoint.latitude = point.y
                waypoint.longitude = point.x
                waypoint.pos_x = 0.0
                waypoint.pos_y = 0.0
                waypoint.pos_z = 0.0
                utm_zone = str(int((waypoint.longitude + 180) / 6) + 1) + ("S" if waypoint.latitude < 0 else "N")
                utm_proj = pyproj.Proj(proj="utm", zone=int(utm_zone[:-1]), ellps="WGS84", south=True if utm_zone[-1] == "S" else False)
                x, y = utm_proj(waypoint.longitude, waypoint.latitude)
                waypoint.utm_zone = utm_zone
                # waypoints[key] = waypoint
                waypoints.append(waypoint)
                tmp_waypoint = Waypoint()
                tmp_waypoint.pos_x = float(x)
                tmp_waypoint.pos_y = float(y)
                initial_x = float(x)
                initial_y = float(y)
                tmp_waypoint.longitude = point.x
                tmp_waypoint.latitude = point.y
                tmp_waypoint.utm_zone = utm_zone
                path = os.path.join(os.path.dirname(self.file_path), "T_utm_topomap_" + os.path.basename(self.file_path))
                write_waypoints(path, [tmp_waypoint].copy())
                # with open(path, "w+") as f:
                #     data = {}
                #     x, y, z = pymap3d.ecef.geodetic2ecef(waypoint.latitude, waypoint.longitude, 0.2)
                #     data["T_utm_topomap"] = {}
                #     data["T_utm_topomap"]["position_x"] = float(x)
                #     data["T_utm_topomap"]["position_y"] = float(y)
                #     yaml.safe_dump(data, f)
                
                    
            else:
                waypoint = Waypoint()
                waypoint.mode = Waypoint.WaypointMode.NORMAL
                waypoint.latitude = point.y
                waypoint.longitude = point.x
                
                # e, n, u = pymap3d.geodetic2enu(waypoint.latitude, waypoint.longitude, 0.0, waypoints[0].latitude, waypoints[0].longitude, 0.0)
                # e, n, h = pymap3d.geodetic2enu(waypoints[0].latitude, waypoints[0].longitude, 0.0, point.y, point.x, 0.0)
                utm_zone = str(int((waypoint.longitude + 180) / 6) + 1) + ("S" if waypoint.latitude < 0 else "N")
                utm_proj = pyproj.Proj(proj="utm", zone=int(utm_zone[:-1]), ellps="WGS84", south=True if utm_zone[-1] == "S" else False)
                x, y = utm_proj(waypoint.longitude, waypoint.latitude)
                e = x - initial_x
                n = y - initial_y

                # result = compute_azi1_enu_with_utm(
                #     waypoints[key - 1].latitude,
                #     waypoints[key - 1].longitude,
                #     point.y,
                #     point.x
                # )
                
                waypoint.pos_x = float(e)
                waypoint.pos_y = float(n)
                waypoint.pos_z = 0.0
                waypoint.utm_zone = utm_zone
                
                # waypoints[key - 1].yaw = result[0]
                waypoints[key - 1].yaw = float(math.atan2(n - waypoints[key - 1].pos_y, e - waypoints[key - 1].pos_x))
                waypoints.append(waypoint)
                # waypoints[key] = waypoint
            
            key += 1
        
        path = os.path.join(os.path.dirname(self.file_path), "T_topomap_liomap_" + os.path.basename(self.file_path))
        index = 5
        send_data = []
        for i in range(index):
            send_data.append((
                waypoints[i].pos_x,
                waypoints[i].pos_y,
                waypoints[i].yaw,
            ))
        # initial_yaw = calculate_initial_yaw(data=send_data, num_points=index)
        # initial_yaw = (waypoints[0].yaw + waypoints[1].yaw) / 2
        initial_yaw = waypoints[0].yaw        
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, initial_yaw)
        tmp_waypoint = Waypoint()
        tmp_waypoint.quat_x = float(q_x)
        tmp_waypoint.quat_y = float(q_y)
        tmp_waypoint.quat_z = float(q_z)
        tmp_waypoint.quat_w = float(q_w)
        tmp_waypoint.yaw = initial_yaw
        write_waypoints(path, [tmp_waypoint].copy())
        # data.clear()
        # with open(path, "w+") as f:            
        #     data["T_topomap_liomap"] = {}
        #     data["T_topomap_liomap"]["position_x"] = 0.0
        #     data["T_topomap_liomap"]["position_y"] = 0.0
        #     data["T_topomap_liomap"]["position_z"] = 0.0
        #     data["T_topomap_liomap"]["quaternion_x"] = float(q_x)
        #     data["T_topomap_liomap"]["quaternion_y"] = float(q_y)
        #     data["T_topomap_liomap"]["quaternion_z"] = float(q_z)
        #     data["T_topomap_liomap"]["quaternion_w"] = float(q_w)
        #     data["T_topomap_liomap"]["roll"] = 0.0
        #     data["T_topomap_liomap"]["pitch"] = 0.0
        #     data["T_topomap_liomap"]["yaw"] = initial_yaw

        #     yaml.safe_dump(data, f)

        write_path = ""
        dir = os.path.dirname(self.file_path)
        name = os.path.basename(self.file_path)

        if self.file_number == 0:
            write_path = os.path.join(dir, name)
        else:
            write_path = os.path.join(dir, str(self.file_number) + "_" + name)
        
        # write_data = {}
        # for key in waypoints:
        #     write_data[key] = {}
        #     write_data[key]["position_x"] = waypoints[key].pos_x
        #     write_data[key]["position_y"] = waypoints[key].pos_y
        #     write_data[key]["position_z"] = waypoints[key].pos_z
        #     write_data[key]["quaternion_x"] = waypoints[key].quat_x
        #     write_data[key]["quaternion_y"] = waypoints[key].quat_y
        #     write_data[key]["quaternion_z"] = waypoints[key].quat_z
        #     write_data[key]["quaternion_w"] = waypoints[key].quat_w
        #     write_data[key]["roll"] = waypoints[key].roll
        #     write_data[key]["pitch"] = waypoints[key].pitch
        #     write_data[key]["yaw"] = waypoints[key].yaw
        #     write_data[key]["longitude"] = waypoints[key].longitude
        #     write_data[key]["latitude"] = waypoints[key].latitude
        #     write_data[key]["mode"] = waypoints[key].mode
        #     write_data[key]["covariance"] = waypoints[key].covariance.copy()
        # with open(write_path, "w+") as f_operator:
        #     yaml.safe_dump(data=write_data, stream=f_operator)
        #     self.get_logger().info("Topological Map is saved into the path given!")
        # for waypoint in waypoints:
        #     self.get_logger().info("{}".format(waypoint.pos_x, waypoint.pos_y, waypoint.pos_z, waypoint.yaw, waypoint.utm_zone))
        write_waypoints(write_path, waypoints)
        self.get_logger().info("Topological Map is saved into the path given!")

        waypoints.clear()

    def mv_kml_file(self, msg : String):
        time_str = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime(float(self.get_clock().now().nanoseconds * 1e-9)))
        os.rename(msg.data, os.path.join(os.path.dirname(self.file_path), time_str + "_" +"GeoPoints.kml"))

def main():
    rclpy.init(args=sys.argv)
    geo_topological_map = GeoTopologicalMap()
    rclpy.spin(geo_topological_map)
    rclpy.shutdown()

if __name__ == "__main__":
    main()