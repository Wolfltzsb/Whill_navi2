from whill_navi2.modules.waypoint_pkg_utils import Waypoint, read_waypoints, write_waypoints
import yaml, csv, sys, os, pyproj, numpy
import tf_transformations

# def calculate_initial_yaw(data : list, num_points : int = 10) -> float:
#     if len(data) < 2:
#         raise ValueError("Point is not enough.")
    
#     initial_points = data[:num_points]
#     x_coords = numpy.array([point[0] for point in initial_points])
#     y_coords = numpy.array([point[1] for point in initial_points])
    
#     # 使用线性回归拟合直线
#     coefficients = numpy.polyfit(x_coords, y_coords, 1)  # 一次多项式拟合
#     slope = coefficients[0]  # 斜率
    
#     # 计算yaw (方向角)
#     initial_yaw = numpy.arctan2(y_coords[-1] - y_coords[0], x_coords[-1] - x_coords[0])
    
#     return float(initial_yaw)

# waypoints = read_waypoints("/mnt/shared/data/rinpu2/topological_map.yaml")
# initial_yaw = calculate_initial_yaw([(waypoint.pos_x, waypoint.pos_y) for waypoint in waypoints], num_points=7)
# print(initial_yaw)

# print(sys.argv)
if len(sys.argv) < 8:
    print("Usage: convert_waypoint_to_csv.py [input_file_path] [x, y, z, roll, pitch, yaw]")
    exit(1)
else:
    input_file_path = sys.argv[1]
    transform = []
    for i in sys.argv[2:]:
        transform.append(float(i))

# waypoints = read_waypoints(input_file_path)
waypoints = []

waypoints = read_waypoints(input_file_path)

source_traj = []
for waypoint in waypoints:
    source_traj.append([waypoint.pos_x, waypoint.pos_y, waypoint.pos_z, waypoint.yaw])

rotation_matrix = tf_transformations.euler_matrix(transform[3], transform[4], transform[5])
source_traj = numpy.array(source_traj)
transformed_traj = numpy.dot(source_traj, rotation_matrix.T)

waypoints.clear()
for i in transformed_traj:
    waypoint = Waypoint()
    waypoint.pos_x = float(i[0])
    waypoint.pos_y = float(i[1])
    waypoint.posz = float(i[2])
    waypoint.yaw = float(i[3])
    waypoints.append(waypoint)

write_waypoints(os.path.join(os.path.dirname(input_file_path), "transformed_" + os.path.basename(input_file_path)), waypoints)