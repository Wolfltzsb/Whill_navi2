import rclpy, os, numpy as np, pycurl
from xml.etree import ElementTree
from geometry_msgs.msg import Point, Vector3, Pose
from geographic_msgs.msg import GeoPose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math, yaml

WGS84 = {"a" : 6378137.0, "b" : 6356752.314245, "f" : 1 / 298.257223563}
'''
the parameter of WGS84
'''
    
class Waypoint():
    """
        Description of Waypoint
        
        Attrs:
        - waypoint: a dictionary is like (key : value):
            - pos_x : position_x (float)
            - pos_y : position_y (float)
            - pos_z : position_z (float)
            - quat_x : quaternion_x (float)
            - quat_y : quaternion_y (float)
            - quat_z : quaternion_z (float)
            - quat_w : quaternion_w (float)
            - roll : roll (float)
            - pitch : pitch (float)
            - yaw : yaw (float)
            - longitude : longitude (float)
            - latitude : latitude (float)
            - mode : mode (int)
        - wayponits : a dictionary is like:
            - dict["waypoins" : [{}]]
    """
    class WaypointMode():
        """
            Describe the Mode fo Waypoint with enumeration
            
            Attrs:
            - NORMAL: Noraml navigation mode without specified task
            - SEARCH: Search some objects using image recognition
            - CANCEL: Unkown
            - DIRECT: Unkown
            - STOP: Stop moving when robot has arrived to this waypoint
            - SIGNAL: Execute the task which is the recognition of traffic light
        """
        NORMAL = 0
        SEARCH = 1
        CANCEL = 2
        DIRECT = 3
        STOP = 4
        SIGNAL = 5
        CHANGE_MAP = 6   
    
    pos_x = 0.0
    pos_y = 0.0
    pos_z = 0.0
    quat_x = 0.0
    quat_y = 0.0
    quat_z = 0.0
    quat_w = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    longitude = 0.0
    latitude = 0.0
    utm_zone = "None"
    time_stamp = 0.0
    mode : WaypointMode = WaypointMode.NORMAL
    covariance = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ]

        
def vector3_to_point(vector3 : Vector3) -> Point:
    """
        Convert Vector3 to Point. Both Vector3 and Point is a kind of ROS Message Interface
        
        Args:
        - vector3: geometry_msgs/msg/Vector3

        Return:
        - point: geometry_msgs/msg/Point
    """
    point = Point()
    point.x = vector3.x
    point.y = vector3.y
    point.z = vector3.z
    
    return point

def pose_to_waypoint(pose : Pose, vector : Vector3 = None) -> Waypoint:
    """
        According to geometry_msgs/msg/Pose and Vector3(optional), generate a waypoint whose message type is dictionary
        
        Args:
        - pose: geometry_msgs/msg/Pose
        - vector: geometry_msgs/msg/Vector3(optional). if specified, the orientation settings will follow it accordingly. 

        Return:
        - waypoint: dictionary with localization of robot such as point, orientation and geopose
    """
    waypoint = Waypoint()
    rpy = euler_from_quaternion([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,        
    ])
    if vector is not None:
        waypoint.pos_x = vector.x
        waypoint.pos_y = vector.y
        waypoint.pos_z = vector.z
    else:
        waypoint.pos_x = pose.position.x
        waypoint.pos_y = pose.position.y
        waypoint.pos_z = pose.position.z
        
    waypoint.quat_x = pose.orientation.x
    waypoint.quat_y = pose.orientation.y
    waypoint.quat_z = pose.orientation.z
    waypoint.quat_w = pose.orientation.w
    waypoint.roll = rpy[0]
    waypoint.pitch = rpy[1]
    waypoint.yaw = rpy[2]
    waypoint.mode = Waypoint.WaypointMode.NORMAL
    
    return waypoint

def get_geolocation(rclpy_node_name : str, place : str) -> list[float] | None:
    """
        Get Geolocation from Internet by giving a place name
        
        Args:
        - rclpy_node : A rclpy.node.Node Object for ros_logger
        - place : Place name which will give a Geolocation back
            
        Return: A list with Geolocation
        - [Latitude, Longitude]
    """   
    
    c = pycurl.Curl()

    if " " in place:
        place = place.split(" ")
        place = "%20".join(place)
        
    c.setopt(pycurl.URL, "https://www.geocoding.jp/api/?q={}".format(place))

    if os.environ["http_proxy"] is not None:
        c.setopt(pycurl.PROXY, os.environ["http_proxy"])
        
    s = c.perform_rs()
    
    if s == None:
        rclpy.node.get_logger(rclpy_node_name).info("Couldn't connect to URL, Please check the Network")
        return None
    
    xml_root = ElementTree.fromstring(s)
    
    result = []

    for coordinate in xml_root.iter('coordinate'):
        for lat in coordinate.iter('lat'):
            result.append(float(lat.text))
        for lng in coordinate.iter('lng'):
            result.append(float(lng.text))
            
    return result

def read_waypoints(waypoint_file_path : str) -> list[Waypoint] | bool:
    waypoints = []
    if os.path.exists(waypoint_file_path):
        with open(waypoint_file_path, "r") as f:
            documents = yaml.safe_load_all(f)
            for document in documents:
                for key in document:
                    waypoint = Waypoint()
                    waypoint.pos_x = float(document[key]["position_x"])
                    waypoint.pos_y = float(document[key]["position_y"])
                    waypoint.pos_z = float(document[key]["position_z"])
                    waypoint.quat_x = float(document[key]["quaternion_x"])
                    waypoint.quat_y = float(document[key]["quaternion_y"])
                    waypoint.quat_z = float(document[key]["quaternion_z"])
                    waypoint.quat_w = float(document[key]["quaternion_w"])
                    waypoint.roll = float(document[key]["roll"])
                    waypoint.pitch = float(document[key]["pitch"])
                    waypoint.yaw = float(document[key]["yaw"])
                    waypoint.longitude = float(document[key]["longitude"])
                    waypoint.latitude = float(document[key]["latitude"])
                    waypoint.mode = float(document[key]["mode"])
                    if "utm_zone" in document[key].keys():      
                        waypoint.utm_zone = str(document[key]["utm_zone"])
                    if "time_stamp" in document[key].keys():
                        waypoint.time_stamp = float(document[key]["time_stamp"])
                    waypoint.covariance.clear()
                    if "covariance" in document[key].keys():
                        for elem in document[key]["covariance"]:
                            waypoint.covariance.append(elem)
                    
                    waypoints.append(waypoint)
        return waypoints
    else:
        return False

def write_waypoints(waypoint_file_path : str, waypoints : list[Waypoint]):
    documents = []
    with open(waypoint_file_path, "w+") as f:
        index = 0
        for waypoint in waypoints:
            document = {}
            document[index] = {}
            document[index]["position_x"] = waypoint.pos_x
            document[index]["position_y"] = waypoint.pos_y
            document[index]["position_z"] = waypoint.pos_z
            document[index]["quaternion_x"] = waypoint.quat_x
            document[index]["quaternion_y"] = waypoint.quat_y
            document[index]["quaternion_z"] = waypoint.quat_w
            document[index]["quaternion_w"] = waypoint.quat_z
            document[index]["roll"] = waypoint.roll
            document[index]["pitch"] = waypoint.pitch
            document[index]["yaw"] = waypoint.yaw
            document[index]["pitch"] = waypoint.pitch
            document[index]["longitude"] = waypoint.longitude
            document[index]["latitude"] = waypoint.latitude
            document[index]["mode"] = waypoint.mode
            document[index]["utm_zone"] = waypoint.utm_zone
            document[index]["time_stamp"] = waypoint.time_stamp
            document[index]["covariance"] = []
            for elem in waypoint.covariance:
                document[index]["covariance"].append(elem)
            documents.append(document)
            
            index += 1
        
        yaml.safe_dump_all(documents, f)

