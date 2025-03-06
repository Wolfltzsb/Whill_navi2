import rclpy, pymap3d
from rclpy.node import Node
import rclpy.qos
from tf2_geometry_msgs.tf2_geometry_msgs import TransformStamped
import tf2_geometry_msgs, pyproj
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration as RosDuration
from rclpy.time import Time as RosTime
import tf_transformations, math, tf2_ros
from robot_localization.srv import SetPose
from sensor_msgs.msg import PointCloud2

class TransformOdom(Node):
    def __init__(self):
        super().__init__("transform_odom_node")
        
        # TF2 Buffer and Listener
        # self.tf2_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf2_buffer, self, spin_thread=True)

        # Transform Broadcaster
        # self.tf_broadcaster = TransformBroadcaster(self)

        # Subscription to global odometry (map->base_link)
        self.create_subscription(
            Odometry, '/glim_rosnode/odom', self.odom0_callback, 5
        )
        self.create_subscription(
            Odometry, '/zed2i/zed_node/odom', self.odom1_callback, 5
        )
        # self.create_subscription(
        #     Odometry, '/odometry/filtered/global', self.odom2_callback, 5
        # )
        self.create_subscription(
            Odometry, '/whill/odometry', self.odom3_callback, 5
        )
        self.create_subscription(
            Imu, '/zed2i/zed_node/imu/data_enu', self.imu0_callback, 5
        )
        self.create_subscription(
            NavSatFix, '/gnss/fix', self.on_gnss_callback, 5
        )
        self.create_subscription(
            PointCloud2, "/globalmap", self.on_pointcloud_callback, 1
        )
        self.gnss_data_queue = []
        self.x_lio_before = 0.0
        self.y_lio_before = 0.0
        self.index = 0.0
                
        # qos = rclpy.qos.QoSProfile(
        #         depth=1,
        #         durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        #         history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        #         )

        # self.static_tf_publisher = self.create_publisher(TFMessage, "/tf_static", qos)
        # msgs = TFMessage()
        # msg = TransformStamped()
        # msg.header.frame_id = "utm"
        # # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.child_frame_id = "gnss_map"
        # msg.transform.translation.x = 50.0
        # msg.transform.translation.y = 50.0
        # msgs.transforms.append(msg)
        
        # self.static_tf_publisher.publish(msgs)
        self.tf2_buffer = tf2_ros.buffer.Buffer()
        self.tf2_listener = tf2_ros.transform_listener.TransformListener(buffer=self.tf2_buffer, node=self, spin_thread=True)
        self.tf2_static_broadcaster = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster(node=self)
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.covariance_list = []
        
        longitude = 135.51181030273438
        latitude = 34.77418899536133
        utm_proj = pyproj.Proj(proj="utm", zone=(int((longitude + 180) / 6) + 1), ellps="WGS84", south=latitude < 0)
        x, y = utm_proj(longitude, latitude)
        
        # transform = self.tf2_buffer.lookup_transform("utm", "gnss_map", time=RosTime())
        # transform.transform.translation.x = 50.0
        # transform.transform.translation.y = 50.0
        # transform.header.stamp = self.get_clock().now().to_msg()
        # tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster(self, 10).pub_tf(transform)

        self.transformed_odom0_pub = self.create_publisher(Odometry, '/glim_rosnode/odom/transformed', 5)
        self.transformed_odom1_pub = self.create_publisher(Odometry, '/zed2i/zed_node/odom/transformed', 5)
        self.transformed_odom2_pub = self.create_publisher(Odometry, '/odometry/filtered/local/transformed', 5)
        self.transformed_odom3_pub = self.create_publisher(Odometry, '/whill/odometry/transformed', 5)
        
        self.single_gnss_sub = self.create_subscription(Odometry, "/odometry/gps_single", self.on_single_gnss, 10)
        self.rtk_gnss_sub = self.create_subscription(Odometry, "/odometry/gps_rtk", self.on_rtk_gnss, 10)
        self.single_gnss_pub = self.create_publisher(Odometry, "/odometry/gps_single/sim", 10)
        self.rtk_gnss_pub = self.create_publisher(Odometry, "/odometry/gps_rtk/sim", 10)
        
        self.set_pose_client = self.create_client(SetPose, "/glim_rosnode/set_pose")
        # self.timer = self.create_timer(0.2, self.set_pose_callback)
        self.initial_yaw = 0
        self.count = 0
        
        self.transform = TransformStamped()
        # self.transform.transform.rotation.z = 0.20362991478500414
        # self.transform.transform.rotation.w = 0.9790479343753563

        # self.transform.transform.rotation.z = 0.19990373231612163
        # self.transform.transform.rotation.w = 0.9798155427457172
        
        self.transform.transform.rotation.z = -0.9964823993436389
        self.transform.transform.rotation.w = 0.08380231380066096
        
        self.test_tf_static_rate_timer = self.create_timer(0.0001, self.test_tf_static_callback)

    def test_tf_static_callback(self):
        transform = TransformStamped()
        transform.header.frame_id = "lio_odom"
        # transform.header.stamp = self.get_clock().now().to_msg()
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.index
        transform.transform.translation.y = self.index
        self.tf2_static_broadcaster.sendTransform(transform)
        self.index = self.index + 1
        print("send")
    
    def on_pointcloud_callback(self, msg : PointCloud2):
        
        print(int(msg.height * msg.width))
   
    def odom0_callback(self, msg : Odometry):
        # pose = PoseWithCovarianceStamped()
        # pose.header = msg.header
        # pose.pose = msg.pose
        # transformed_pose = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(pose, self.transform)
        # msg.pose = transformed_pose.pose
        # msg.header = transformed_pose.header
        msg.header.frame_id = "lio_vio_odom"

        
        self.transformed_odom0_pub.publish(msg)
    
    def odom1_callback(self, msg : Odometry):
        # pose = PoseWithCovarianceStamped()
        # pose.header = msg.header
        # pose.pose = msg.pose
        # transformed_pose = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(pose, self.transform)
        # msg.pose = transformed_pose.pose
        # msg.header = transformed_pose.header
        msg.header.frame_id = "vio_odom"
        
        self.transformed_odom1_pub.publish(msg)
    
    def odom2_callback(self, msg : Odometry):
        if len(self.covariance_list) < 1200:
            covariance = math.hypot(msg.pose.covariance[0], msg.pose.covariance[7])
            if covariance >= 1.0:
                covariance = covariance * 10
            else:
                covariance = covariance
            self.covariance_list.append(covariance)
            if len(self.covariance_list) > 0:
                average_covariance = sum(self.covariance_list) / len(self.covariance_list)
        else:
            self.covariance_list.pop(0)
            self.covariance_list.append(math.hypot(msg.pose.covariance[0], msg.pose.covariance[7]))
            average_covariance = sum(self.covariance_list) / len(self.covariance_list)
        
        if self.tf2_buffer.can_transform("transformed_lio_map", "base_link", time=RosTime()):
            transform = self.tf2_buffer.lookup_transform("transformed_lio_map", "base_link", time=RosTime())
            if math.hypot((transform.transform.translation.x - msg.pose.pose.position.x), (transform.transform.translation.y - msg.pose.pose.position.y)) >= 1.0 and len(self.covariance_list) >= 200 and average_covariance <= 20.0:
                T_transformed_lio_map_lio_map = self.tf2_buffer.lookup_transform("lio_map", "transformed_lio_map", time=RosTime())
                T_transformed_lio_map_lio_map.transform.translation.x -= msg.pose.pose.position.x - transform.transform.translation.x
                T_transformed_lio_map_lio_map.transform.translation.y -= msg.pose.pose.position.y - transform.transform.translation.y
                self.tf_static_broadcaster.sendTransform(T_transformed_lio_map_lio_map)
            # print(math.hypot((transform.transform.translation.x - msg.pose.pose.position.x), (transform.transform.translation.y - msg.pose.pose.position.y)), average_covariance)
        # pose = PoseWithCovarianceStamped()
        # pose.header = msg.header
        # pose.pose = msg.pose
        # transformed_pose = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(pose, self.transform)
        # msg.pose = transformed_pose.pose
        # msg.header = transformed_pose.header
        
        self.transformed_odom2_pub.publish(msg)
    
    def on_single_gnss(self, msg : Odometry):
        odom = Odometry()
        odom = msg
        odom.header.stamp = self.get_clock().now().to_msg()
        
        self.single_gnss_pub.publish(odom)
    def on_rtk_gnss(self, msg : Odometry):
        odom = Odometry()
        odom = msg
        odom.header.stamp = self.get_clock().now().to_msg()
        
        self.rtk_gnss_pub.publish(odom)
        
    def odom3_callback(self, msg : Odometry):
        # pose = PoseWithCovarianceStamped()
        # pose.header = msg.header
        # pose.pose = msg.pose
        # transformed_pose = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(pose, self.transform)
        # msg.pose = transformed_pose.pose
        # msg.header = transformed_pose.header
        msg.header.frame_id = "whill_odom"
        
        self.transformed_odom3_pub.publish(msg)
    
    def imu0_callback(self, msg : Imu):
        yaw = tf_transformations.euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])[2] / math.pi * 180

        # print(yaw)
    
    def set_pose_callback(self):
        req = SetPose.Request()
        # self.set_pose_client.wait_for_service(0.1)
        x, y, z, w = tf_transformations.quaternion_from_euler(0.0, 0.0, self.initial_yaw)
        req.pose.pose.pose.orientation.x = float(x)
        req.pose.pose.pose.orientation.y = float(y)
        req.pose.pose.pose.orientation.z = float(z)
        req.pose.pose.pose.orientation.w = float(w)
        req.pose.header.stamp = self.get_clock().now().to_msg()
        
        self.set_pose_client.call_async(req)
        self.initial_yaw += 10
        
        if self.initial_yaw > 360:
            self.initial_yaw = 0

    def on_gnss_callback(self, msg : NavSatFix):
        delta_dis_gnss = 0.0
        delta_dis_lio = 0.0

        x, y, h = pymap3d.enu.geodetic2enu(msg.latitude, msg.longitude, 0.0, 34.774200166666667, 135.51181183333333, 0.0)
        covariance = msg.position_covariance[0]
        
        if len(self.gnss_data_queue) == 0:
            yaw = 0.42
        else:
            yaw = math.atan2(y - self.gnss_data_queue[-1][1], x - self.gnss_data_queue[-1][0])
            delta_dis_gnss = math.hypot((self.gnss_data_queue[-1][0] - x), (self.gnss_data_queue[-1][1] - y))
            # if len(self.gnss_data_queue) == 10:
            #     if self.tf2_buffer.can_transform("lio_map", "transformed_lio_map", time=RosTime(), timeout=RosDuration(seconds=1)):
            #         qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, - self.gnss_data_queue[-1][2])
            #         T_topomap_lio_map = self.tf2_buffer.lookup_transform("lio_map", "transformed_lio_map", time=RosTime())
            #         T_topomap_lio_map.transform.rotation.x = qx
            #         T_topomap_lio_map.transform.rotation.y = qy
            #         T_topomap_lio_map.transform.rotation.z = qz
            #         T_topomap_lio_map.transform.rotation.w = qw
            #         self.tf2_static_broadcaster.sendTransform(T_topomap_lio_map)
                    
        
        if len(self.gnss_data_queue) < 60:
            self.gnss_data_queue.append((x, y, yaw, covariance))
        else:
            self.gnss_data_queue.pop(0)
            self.gnss_data_queue.append((x, y, yaw, covariance))
        
        covariances = []
        bad_covariance_count = 0
        for gnss_data in self.gnss_data_queue:
            covariances.append(gnss_data[3])
            if gnss_data[3] >= 10.0:
                bad_covariance_count += 1
        
        average_covariance = sum(covariances) / len(covariances)
        can_T = self.tf2_buffer.can_transform("topomap", "base_link", time=RosTime())
        delta_yaw = 0.0
        if can_T:
            transform = self.tf2_buffer.lookup_transform("topomap", "base_link", time=RosTime())
            if len(covariances) >= 60 and average_covariance <= 5.0 and bad_covariance_count <= 2:
                if math.hypot((transform.transform.translation.x - x), (transform.transform.translation.y - y)) >= 1.0:
                    T_topomap_lio_map = self.tf2_buffer.lookup_transform("topomap", "lio_map", time=RosTime())
                    T_topomap_lio_map.transform.translation.x += x - transform.transform.translation.x
                    T_topomap_lio_map.transform.translation.y += y - transform.transform.translation.y
                    delta_dis_lio = math.hypot((T_topomap_lio_map.transform.translation.x - self.x_lio_before), (T_topomap_lio_map.transform.translation.y - self.y_lio_before))
                    self.x_lio_before = T_topomap_lio_map.transform.translation.x
                    self.y_lio_before = T_topomap_lio_map.transform.translation.y
                    
                    # if abs(delta_dis_lio - delta_dis_gnss) <= 1.6666:
                    #     self.tf2_static_broadcaster.sendTransform(T_topomap_lio_map)
                    self.tf2_static_broadcaster.sendTransform(T_topomap_lio_map)
                
                lio_yaw = tf_transformations.euler_from_quaternion([
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ])[2]
                
                delta_yaw = yaw - lio_yaw
                if abs(math.degrees(delta_yaw)) >= 5.0 :
                    # req = SetPose.Request()
                    # send_yaw = lio_yaw + delta_yaw / 2.0
                    # q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, send_yaw)
                    # req.pose.pose.pose.orientation.x = q_x
                    # req.pose.pose.pose.orientation.y = q_y
                    # req.pose.pose.pose.orientation.z = q_z
                    # req.pose.pose.pose.orientation.w = q_w
                    
                    # self.set_pose_client.call_async(req)
                    
                    # T_lio_map_lio_odom = self.tf2_buffer.lookup_transform("lio_map", "lio_odom", time=RosTime())
                    # T_lio_map_lio_odom.transform.rotation.x = q_x
                    # T_lio_map_lio_odom.transform.rotation.y = q_y
                    # T_lio_map_lio_odom.transform.rotation.z = q_z
                    # T_lio_map_lio_odom.transform.rotation.w = q_w
                    # self.tf2_broadcaster.sendTransform(T_lio_map_lio_odom)
                    
                    print(delta_yaw)
            
            print(x, y, delta_yaw, average_covariance, bad_covariance_count, delta_dis_lio, delta_dis_gnss)

        
        covariances.clear()

    
    
    def compute_and_publish_transform(self):
        if not self.latest_map_to_base_link:
            return

        try:
            # Get the latest odom->base_link transform
            odom_to_base_link = self.tf2_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )

            # Convert map->base_link to a transform
            map_to_base_link = self.latest_map_to_base_link

            # Compute map->odom = map->base_link * inverse(odom->base_link)
            map_to_odom = self.compute_map_to_odom(map_to_base_link, odom_to_base_link)

            # Publish map->odom transform
            self.publish_transform(map_to_odom)

        except Exception as e:
            self.get_logger().warn(f"Failed to lookup transform: {e}")

    def compute_map_to_odom(self, map_to_base_link, odom_to_base_link):
        # Convert poses to transformation matrices
        map_to_base_link_mat = tf_transformations.translation_matrix([
            map_to_base_link.position.x,
            map_to_base_link.position.y,
            map_to_base_link.position.z
        ])
        map_to_base_link_rot = tf_transformations.quaternion_matrix([
            map_to_base_link.orientation.x,
            map_to_base_link.orientation.y,
            map_to_base_link.orientation.z,
            map_to_base_link.orientation.w
        ])
        map_to_base_link_mat[:3, :3] = map_to_base_link_rot[:3, :3]

        odom_to_base_link_mat = tf_transformations.translation_matrix([
            odom_to_base_link.transform.translation.x,
            odom_to_base_link.transform.translation.y,
            odom_to_base_link.transform.translation.z
        ])
        odom_to_base_link_rot = tf_transformations.quaternion_matrix([
            odom_to_base_link.transform.rotation.x,
            odom_to_base_link.transform.rotation.y,
            odom_to_base_link.transform.rotation.z,
            odom_to_base_link.transform.rotation.w
        ])
        odom_to_base_link_mat[:3, :3] = odom_to_base_link_rot[:3, :3]

        # Compute map->odom transform
        map_to_odom_mat = map_to_base_link_mat @ tf_transformations.inverse_matrix(odom_to_base_link_mat)

        # Extract translation and rotation
        translation = tf_transformations.translation_from_matrix(map_to_odom_mat)
        rotation = tf_transformations.quaternion_from_matrix(map_to_odom_mat)

        # Create TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        return transform

    def publish_transform(self, transform):
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = TransformOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()