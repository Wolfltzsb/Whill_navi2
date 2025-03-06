from whill_navi2.modules.waypoint_pkg_utils import (
    rclpy, quaternion_from_euler, os, Waypoint,
    read_waypoints
)
from rclpy.duration import Duration
from rclpy.time import Time
import yaml, math
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Twist
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Empty
from hdl_global_localization.srv import SetGlobalLocalizationEngine
from tf2_ros.transform_listener import Buffer, TransformListener

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    # PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
]


class WhillNavi2Node(BasicNavigator):
    
    def __init__(self):
        super().__init__(node_name="whill_navi2_node")
        
        self.waypoints_all = []
        self.waypoint_file_paths = self.declare_parameter(
            "waypoint_file_paths", 
            [os.path.join(os.environ["HOME"], "Documents", "test_waypoint_recorder_rewrite.yaml")]
        ).get_parameter_value().string_array_value
        self.pcd_file_paths = self.declare_parameter(
            "pcd_file_paths", 
            [os.path.join(os.environ["HOME"], "Documents", "test.pcd")]
        ).get_parameter_value().string_array_value
        self.gridmap_file_paths = self.declare_parameter(
            "gridmap_file_paths", 
            [os.path.join(os.environ["HOME"], "Documents", "test_waypoint_recorder_rewrite.yaml")]
        ).get_parameter_value().string_array_value
        self.is_loop_mode = self.declare_parameter(
            'is_loop_mode', False
        ).get_parameter_value().bool_value
        self.loop_limit = self.declare_parameter(
            'loop_limit', 2
        ).get_parameter_value().integer_value
        self.use_map = self.declare_parameter(
            "use_map", True
        ).get_parameter_value().bool_value
        self.map_frame_id = self.declare_parameter(
            "map_frame_id", "map"
        ).get_parameter_value().string_value
        self.use_remap = self.declare_parameter(
            "use_remap", False
        ).get_parameter_value().bool_value

        self.waypoints_marker_pub = self.create_publisher(MarkerArray, "waypoints_markers", 5)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 5)
        self.set_pcd_map_pub = self.create_publisher(String, "map_request/pcd", 1)
        self.relocalize_client = self.create_client(Empty, "hdl_localization/relocalize")
        self.set_global_engine_client = self.create_client(SetGlobalLocalizationEngine, "set_engine")
        self.stop_signal_sub = self.create_subscription(String, "lidar_stop/stop", self.stop_sub_callback, 2)
        self.tf_buffer = Buffer(Duration(), self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        # self.current_waypoint_number_pub = self.create_publisher(Int16, "whill_navi2_node/current_waypoint_number", 3)
        # self.lumotive_pointcloud_sub = self.create_subscription(PointCloud2, "lumotive_ros/pointcloud", self.pointcloud_callback, 10)
        self.velocity_pub = self.create_publisher(Twist, "whill/controller/cmd_vel", 30)
        # self.o3d_pcd = open3d.geometry.PointCloud()
        # self.stop_flag = False
        
        # self.round_number = 1
        self.stop_flag = False
        for waypoint_file_path in self.waypoint_file_paths:
            waypoints = read_waypoints(waypoint_file_path)
            
            self.waypoints_all.append(waypoints)

        self.get_logger().info("Get {} waypoint files".format(str(len(self.waypoints_all))))
                
    
    def publish_waypoint_markers(self, waypoints : list[Waypoint]):
        array_arrow, array_text = MarkerArray(), MarkerArray()        
        
        for index in range(1, len(waypoints)):
            
            arrow, text = Marker(), Marker()
            
            arrow.header.frame_id = text.header.frame_id = self.map_frame_id
            arrow.header.stamp = text.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = "ARROW"
            text.ns = "TEXT"
            
            arrow.color.b = text.color.b = 1.0
            text.color.g = 1.0
            text.color.r = 1.0
            arrow.color.a = text.color.a = 1.0
            arrow.scale.x = 1.0
            arrow.scale.y = 0.2

            arrow.scale.z = text.scale.z = 0.4
            arrow.action = text.action = Marker.ADD
            arrow.type = Marker.ARROW
            text.type = Marker.TEXT_VIEW_FACING
            
            pose_tmp = Pose()
            pose_tmp.position.x = waypoints[index].pos_x
            pose_tmp.position.y = waypoints[index].pos_y
            pose_tmp.position.z = 0.2
            quat = quaternion_from_euler(0.0, 0.0, waypoints[index].yaw)
            pose_tmp.orientation.x = quat[0]
            pose_tmp.orientation.y = quat[1]
            pose_tmp.orientation.z = quat[2]
            pose_tmp.orientation.w = quat[3]
            
            arrow.pose = text.pose = pose_tmp
            text.pose.position.z = 0.5
            arrow.id = text.id = index
            
            text.text = "No.{}-mode.{}".format(index, waypoints[index].mode)
            
            if waypoints[index].mode == Waypoint.WaypointMode.NORMAL:
                text.color.r = 1.0
            elif waypoints[index].mode == Waypoint.WaypointMode.SEARCH:
                text.color.r = 0.0
            
            array_arrow.markers.append(arrow)
            array_text.markers.append(text)
    
        self.waypoints_marker_pub.publish(array_arrow)
        self.waypoints_marker_pub.publish(array_text)
    
    def change_map(self, pcd_map_path : str, gridmap_path : str = "", waypoint : Waypoint = False):    
    # def change_map(self, pcd_map_path : str, waypoint : Waypoint | None):    
        send_data = String()
        send_data.data = pcd_map_path
        if waypoint:
            waypoint.pos_x = 0.0
            waypoint.pos_y = 0.0
            waypoint.pos_z = 0.0
            waypoint.quat_x = 0.0
            waypoint.quat_y = 0.0
            waypoint.quat_z = 0.0
            waypoint.quat_w = 1.0
            self.initial_waypoint(waypoint)

        self.set_pcd_map_pub.publish(send_data)
        if self.use_remap:
            self.changeMap(gridmap_path)
                
        # self.relocalize_client.wait_for_service()
        # relocalize_future = self.relocalize_client.call_async(Empty.Request())
        # rclpy.spin_until_future_complete(self, relocalize_future)

    
    def initial_waypoint(self, waypoint : Waypoint):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = waypoint.pos_x
        initial_pose.pose.position.x = waypoint.pos_y
        initial_pose.pose.position.x = waypoint.pos_z
        initial_pose.pose.orientation.x = waypoint.quat_x
        initial_pose.pose.orientation.y = waypoint.quat_y
        initial_pose.pose.orientation.z = waypoint.quat_z
        initial_pose.pose.orientation.w = waypoint.quat_w
        self.setInitialPose(initial_pose)

    def stop_sub_callback(self, msg : String):
        direction = msg.data.split("/")[0]
        self.stop_angle = float(msg.data.split("/")[1])
        self.stop_flag = True
        
        # self.cancelTask()        
        # self.spin(spin_dist=math.radians(angle) / 2.0, time_allowance=1)
        # self.get_clock().sleep_for(Duration(seconds=1, nanoseconds=int(5e8)))

        return
    
    def dis_from_next(self, waypoint_now : Waypoint | None) -> bool | float:
        if waypoint_now == None:
            return False
        elif self.tf_buffer.can_transform(self.map_frame_id, "base_link", Time()):
            transform_now = self.tf_buffer.lookup_transform(self.map_frame_id, "base_link", Time())
            x_now = transform_now.transform.translation.x
            y_now = transform_now.transform.translation.y
            goal_x = waypoint_now.pos_x
            goal_y = waypoint_now.pos_y
            
            return math.hypot(abs(x_now - goal_x), abs(y_now - goal_y))
        else:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = WhillNavi2Node()
    
    # Initialization
    waypoint_num_now = 1
    loop_now = 1
    waypoints_all_backup = []
    pcd_file_paths_backup = []
    gridmap_file_paths_backup = []
    waypoints_all_backup = []
    waypoints_now = node.waypoints_all.pop(0)
    waypoints_all_backup.append(waypoints_now)
    node.initial_waypoint(waypoints_now[0])
    node.publish_waypoint_markers(waypoints_now)
    node.get_clock().sleep_for(Duration(seconds=1, nanoseconds=0))

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # node.waitUntilNav2Active(navigator="bt_navigator", localizer='hdl_localization_node')
    while not (("hdl_localization_node" in node.get_node_names() and "bt_navigator" in node.get_node_names()) or ("lio_gnss_localization_node" in node.get_node_names() and "bt_navigator" in node.get_node_names())):
        node.get_logger().info("Waitting for Navigator and Localizer ready...")
        node.get_clock().sleep_for(Duration(seconds=1, nanoseconds=0))
    
    if node.use_map:
        node.set_global_engine_client.wait_for_service()
        request = SetGlobalLocalizationEngine.Request()
        request.engine_name.data = "FPFH_RANSAC"
        node.set_global_engine_client.call_async(request)
    
    # If desired, you can change or load the map as well
    # node.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()
    node.get_logger().info("Wait 2 seconds until all ready...")
    node.get_clock().sleep_for(Duration(seconds=2, nanoseconds=0))
    
    while rclpy.ok():        
        # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = node.map_frame_id
        goal_pose.header.stamp = node.get_clock().now().to_msg()
        goal_pose.pose.position.x = waypoints_now[waypoint_num_now].pos_x
        goal_pose.pose.position.y = waypoints_now[waypoint_num_now].pos_y
        quat = quaternion_from_euler(0.0, 0.0, waypoints_now[waypoint_num_now].yaw)
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]
        node.goToPose(goal_pose)

        # sanity check a valid path exists
        # node.getPath(initial_pose, goal_pose)

        # node.goThroughPoses([goal_pose])
        # node.followWaypoints([goal_pose])
        
        # current_waypoint_number_msg = Int16()
        # current_waypoint_number_msg.data = waypoint_num_now - 1
        # node.current_waypoint_number_pub.publish(current_waypoint_number_msg)
        index = 0
        while not node.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            # i = i + 1
            # feedback = node.getFeedback()
            # if feedback and i % 5 == 0:
            #     node.get_logger().info(
            #         'Estimated time of arrival: '
            #         + '{0:.0f}'.format(
            #             Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
            #             / 1e9
            #         )
            #         + ' seconds.'
            #     )

            feedback = node.getFeedback()
            index += 1
            
            if node.stop_flag:
                node.cancelTask()
                for i in range(50):
                    node.get_clock().sleep_for(Duration(seconds=0, nanoseconds=int(1e7)))
                    cmd_vel = Twist()
                    if node.stop_angle >= 0.0:
                        cmd_vel.angular.z = - 0.1
                    elif node.stop_angle < 0.0:
                        cmd_vel.angular.z = 0.1
                    node.velocity_pub.publish(cmd_vel)
                node.stop_flag = False
            
            # if index >= 60:
            #     node.cancelTask()
            #     node.clearLocalCostmap()
            # for i in range(waypoint_num_now, waypoint_num_now + 3):
            #     if i > len(waypoints_now) - 1:
            #         break
            #     if waypoints_now[i].mode != Waypoint.WaypointMode.NORMAL:
            #         special_mode_nearby = True
            #     else:
            #         special_mode_nearby = False
            
            # if feedback and node.dis_from_next(waypoints_now[waypoint_num_now]) <= 0.25 and not special_mode_nearby and len(waypoints_now) - waypoint_num_now > 2:
            #     node.cancelTask()
                
            node.get_logger().info("Navigating, Waypoint number: {}".format(waypoint_num_now))
            rclpy.spin_once(node)
            # node.get_clock().sleep_for(Duration(seconds=0, nanoseconds=int(5e8)))

        # Do something depending on the return code
        result = node.getResult()
        if result == TaskResult.SUCCEEDED:
            node.get_logger().info('Goal succeeded!')
            # if waypoint_num_now == len(node.waypoints):
            #     node.get_logger().info("Arrived at the last waypoint whose number is {}".format(waypoint_num_now - 1))
            #     if node.is_loop_mode:
            #         node.get_logger().info("Success to navigate ROUND {}".format(node.round_number))
            #         node.get_logger().info("A rest until next round...")
            #         node.get_clock().sleep_for(Duration(seconds=10, nanoseconds=0))
            #         waypoint_num_now = 1
            #         node.get_logger().info("Start to navigate ROUND {}".format(node.round_number))
            #     else:
            #         node.get_logger().info("Navigation is over")
            #         break
            print("waypoint number: " + str(waypoint_num_now), ", waypoint number total: " + str(len(waypoints_now)))
            
            if waypoints_now[waypoint_num_now].mode == Waypoint.WaypointMode.CHANGE_MAP:
                next_pcd_map_path = node.pcd_file_paths.pop(0)
                pcd_file_paths_backup.append(next_pcd_map_path)
                if node.use_remap:
                    gridmap_file_paths_backup.append(next_gridmap_path)
                    next_gridmap_path = node.gridmap_file_paths.pop(0)
                node.get_logger().info("Changing to the next pcd map, gridmap...")
                while not node.tf_buffer.can_transform(node.map_frame_id, "base_link", Time()):
                    node.get_logger().warn("waiting for tf ok...")
                    node.get_clock().sleep_for(Duration(seconds=1, nanoseconds=0))
                
                transform = node.tf_buffer.lookup_transform(node.map_frame_id, "base_link", Time())
                waypoint = Waypoint()
                waypoint.pos_x = transform.transform.translation.x
                waypoint.pos_y = transform.transform.translation.y
                waypoint.pos_z = transform.transform.translation.z
                waypoint.quat_x = transform.transform.rotation.x
                waypoint.quat_y = transform.transform.rotation.y
                waypoint.quat_z = transform.transform.rotation.z
                waypoint.quat_w = transform.transform.rotation.w
                # node.change_map(next_pcd_map_path, next_gridmap_path, waypoint)
                node.change_map(next_pcd_map_path, waypoint)
                node.get_clock().sleep_for(Duration(seconds=5, nanoseconds=0))
                waypoint_num_now = waypoint_num_now + 1
            elif waypoints_now[waypoint_num_now].mode == Waypoint.WaypointMode.STOP:
                node.get_logger().warn("Stopping...")
                node.get_logger().info("Waitting for Enter Key...")
                input()
                # node.get_clock().sleep_for(Duration(seconds=6, nanoseconds=0))
                waypoint_num_now += 1
            elif waypoints_now[waypoint_num_now].mode == Waypoint.WaypointMode.NORMAL:
                waypoint_num_now += 1
                node.get_logger().info("Send the next waypiont: {}.".format(waypoint_num_now))
                
            if waypoint_num_now >= len(waypoints_now) - 1:
                if len(node.waypoints_all) == 0:
                    # loop option
                    if node.is_loop_mode and loop_now <= node.loop_limit:
                        # loop
                        node.waypoints_all = waypoints_all_backup
                        
                        loop_now += 1
                    else:
                        break
                
                if node.use_map:
                    if node.use_remap:
                        if len(node.pcd_file_paths) == 0 or len(node.gridmap_file_paths) == 0:
                            node.get_logger().error("No pcd map and grid map for these waypoints, stop navigation")
                            break
                        else:
                            next_pcd_map_path = node.pcd_file_paths.pop(0)
                            next_gridmap_path = node.gridmap_file_paths.pop(0)
                            pcd_file_paths_backup.append(next_pcd_map_path)
                            gridmap_file_paths_backup.append(next_gridmap_path)
                            node.get_logger().info("Changing to the next pcd map, gridmap and waypoints")
                            
                            waypoints_now = node.waypoints_all.pop(0)
                            waypoints_all_backup.append(waypoints_now)
                            waypoint_num_now = 1
                            
                            node.change_map(next_pcd_map_path, next_gridmap_path, waypoints_now[0])
                            # node.change_map(next_pcd_map_path, waypoints_now[0])
                            node.publish_waypoint_markers(waypoints_now)
                            node.get_clock().sleep_for(Duration(seconds=5, nanoseconds=0))
                    else:
                        if len(node.pcd_file_paths) == 0:
                            node.get_logger().error("No pcd map and grid map for these waypoints, stop navigation")
                            break
                        else:
                            next_pcd_map_path = node.pcd_file_paths.pop(0)
                            # next_gridmap_path = node.gridmap_file_paths.pop(0)
                            pcd_file_paths_backup.append(next_pcd_map_path)
                            # gridmap_file_paths_backup.append(next_gridmap_path)
                            node.get_logger().info("Changing to the next pcd map, gridmap and waypoints")
                            
                            waypoints_now = node.waypoints_all.pop(0)
                            waypoints_all_backup.append(waypoints_now)
                            waypoint_num_now = 1
                            
                            # node.change_map(next_pcd_map_path, next_gridmap_path, waypoints_now[0])
                            node.change_map(pcd_map_path=next_pcd_map_path, waypoint=waypoints_now[0])
                            node.publish_waypoint_markers(waypoints_now)
                            node.get_clock().sleep_for(Duration(seconds=5, nanoseconds=0))
                else:
                    waypoints_now = node.waypoints_all.pop(0)
                    waypoints_all_backup.append(waypoints_now)
                    waypoint_num_now = 1
                    node.publish_waypoint_markers(waypoints_now)
                    node.get_clock().sleep_for(Duration(seconds=5, nanoseconds=0))

        elif result == TaskResult.CANCELED:
            print("waypoint number: " + str(waypoint_num_now), ", waypoint number total: " + str(len(waypoints_now)))
            if node.dis_from_next(waypoints_now[waypoint_num_now]):
                if node.dis_from_next(waypoints_now[waypoint_num_now]) and node.dis_from_next(waypoints_now[waypoint_num_now]) > 0.5:
                    waypoint_num_now = waypoint_num_now
            else:
                node.get_logger().warn('Goal was canceled, and send the next waypoint!')
                waypoint_num_now += 1

            if waypoints_now[waypoint_num_now].mode == Waypoint.WaypointMode.CHANGE_MAP and node.use_remap:
                next_pcd_map_path = node.pcd_file_paths.pop(0)
                next_gridmap_path = node.gridmap_file_paths.pop(0)
                pcd_file_paths_backup.append(next_pcd_map_path)
                gridmap_file_paths_backup.append(next_gridmap_path)
                node.get_logger().info("Changing to the next pcd map, gridmap...")                
                node.change_map(next_pcd_map_path, next_gridmap_path, waypoints_now[waypoint_num_now])
                node.get_clock().sleep_for(Duration(seconds=5, nanoseconds=0))
                waypoint_num_now = waypoint_num_now + 1
            else:
                waypoint_num_now = waypoint_num_now + 1
 
        elif result == TaskResult.FAILED:
            node.get_logger().warn('Goal failed!')
        else:
            node.get_logger().error('Goal has an invalid return status!')
    
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
