from whill_navi2.modules.waypoint_pkg_utils import Waypoint, read_waypoints
import yaml, csv, sys, os, pyproj

# print(sys.argv)
if len(sys.argv) < 2:
    print("Usage: convert_waypoint_to_csv.py [input_file_path] [T_utm_topomap_file_path(optional)]")
    exit(1)
elif len(sys.argv) == 2:
    input_file_path = sys.argv[1]
    T_utm_topomap_file_path = ""
    T_utm_topomap_waypoint = {}
elif len(sys.argv) == 3:
    input_file_path = sys.argv[1]
    T_utm_topomap_file_path = sys.argv[2]
    T_utm_topomap_waypoint = read_waypoints(T_utm_topomap_file_path)[0]

# waypoints = read_waypoints(input_file_path)
waypoints = []

with open(input_file_path, "r") as f:
    documents = yaml.safe_load_all(f)
    for document in documents:
        for key in document:
            waypoint = Waypoint()
            waypoint.latitude = float(document[key]["latitude"])
            waypoint.longitude = float(document[key]["longitude"])
            waypoint.pos_x = float(document[key]["position_x"])
            waypoint.pos_y = float(document[key]["position_y"])
            waypoints.append(waypoint)
            
with open(os.path.splitext(input_file_path)[0] + ".csv", "w+") as f:
    fieldnames = ['sequence', 'latitude', 'longitude']
    writer = csv.DictWriter(f, fieldnames=fieldnames)

    sequecne = 0
    writer.writeheader()
    for waypoint in waypoints:
        if waypoint.latitude == waypoint.longitude == 0 and T_utm_topomap_waypoint:
            south = True if T_utm_topomap_waypoint.utm_zone[-1:] == "S" else False
            try:
                utm_zone = int(T_utm_topomap_waypoint.utm_zone[:-1])
            except Exception as e:
                print("Could not get utm_zone: {}".format(utm_zone, e))
                continue
            
            utm_proj = pyproj.Proj(proj="utm", zone=utm_zone, ellps="WGS84", south=south)
            lon, lat = utm_proj(T_utm_topomap_waypoint.pos_x + waypoint.pos_x, T_utm_topomap_waypoint.pos_y + waypoint.pos_y, inverse=True)
            # print(lon, lat)
            writer.writerow({"sequence": sequecne, "latitude" : lat, "longitude": lon})
            sequecne += 1
        else:
            writer.writerow({"sequence": sequecne, "latitude" : waypoint.latitude, "longitude": waypoint.longitude})
            sequecne += 1
            # waypoints[int(key)]["position_x"] = float(document[key]["position_x"])
            # waypoints[int(key)]["position_y"] = float(document[key]["position_y"])
            # waypoints[int(key)]["position_z"] = float(document[key]["position_z"])
            # waypoints[int(key)]["quaternion_x"] = float(document[key]["quaternion_x"])
            # waypoints[int(key)]["quaternion_y"] = float(document[key]["quaternion_y"])
            # waypoints[int(key)]["quaternion_z"] = float(document[key]["quaternion_z"])
            # waypoints[int(key)]["quaternion_w"] = float(document[key]["quaternion_w"])
            # waypoints[int(key)]["roll"] = float(document[key]["roll"])
            # waypoints[int(key)]["pitch"] = float(document[key]["pitch"])
            # waypoints[int(key)]["yaw"] = float(document[key]["yaw"])
            # waypoints[int(key)]["longitude"] = float(document[key]["longitude"])
            # waypoints[int(key)]["latitude"] = float(document[key]["latitude"])
            # waypoints[int(key)]["mode"] = int(document[key]["mode"])
