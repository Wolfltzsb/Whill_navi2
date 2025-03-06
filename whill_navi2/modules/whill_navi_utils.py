from whill_navi2.modules.waypoint_pkg_utils import (
    quaternion_from_euler, GeoPose
)
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from typing import List

def get_poses(waypoints : List[dict], clock_msg : Time) -> List[PoseStamped]:
    '''
        Get the waypoints whose type is a list of a dictionary and the clock message which is a ROS message type. Return a list of pose for navigation.
        
        Args:
            waypoints: A list of waypoint. waypoint : dictionary with number of parameters such as position, orientation and so on.
            clock_msg: ROS Message Type. The time stamp message of this Pose.
            
        Return:
            [pose 1, pose 2, pose 3, ...]. The length of the list is according to waypoints given.
    '''
    goal_poses = []
    for waypoint in waypoints:
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = waypoint["pos_x"]
        goal_pose.pose.position.y = waypoint["pos_y"]
        goal_pose.pose.position.z = waypoint["pos_z"]
        quaternion = quaternion_from_euler(0.0, 0.0, waypoint['yaw'])
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = clock_msg
        goal_poses.append(goal_pose)
    
    return goal_poses

def get_geo_poses(waypoints : List[dict]) -> List[GeoPose] :
    geo_poses = []
    
    for waypoint in waypoints:
        geo_pose = GeoPose()
        geo_pose.position.latitude = waypoint['latitude']
        geo_pose.position.longitude = waypoint['longitude']
        quaternion = quaternion_from_euler(0.0, 0.0, waypoint['yaw'])
        geo_pose.orientation.x = quaternion[0]
        geo_pose.orientation.y = quaternion[1]
        geo_pose.orientation.z = quaternion[2]
        geo_pose.orientation.w = quaternion[3]
        geo_poses.append(geo_pose)
    
    return geo_poses