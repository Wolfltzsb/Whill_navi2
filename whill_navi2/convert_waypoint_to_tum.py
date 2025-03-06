from whill_navi2.modules.waypoint_pkg_utils import Waypoint, read_waypoints
import yaml, csv, sys, os, pyproj
import numpy as np

# print(sys.argv)
if len(sys.argv) < 2:
    print("Usage: convert_waypoint_to_csv.py [input_file_path]")
    exit(1)
elif len(sys.argv) < 3:
    input_file_path = sys.argv[1]
    T_utm_topomap_file_path = ""

# waypoints = read_waypoints(input_file_path)
waypoints = read_waypoints(input_file_path)
poses = []
for waypoint in waypoints:
    poses.append([waypoint.time_stamp, waypoint.pos_x, waypoint.pos_y, 0.0, waypoint.quat_x, waypoint.quat_y, waypoint.quat_z, waypoint.quat_w])

poses = np.array(poses)
path = os.path.splitext(input_file_path)[0] + "_tum.txt"
# print(path)
np.savetxt(path, poses, fmt="%.6f")