import rclpy.duration
from whill_navi2.modules.waypoint_pkg_utils import get_geolocation
import sys
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import rclpy
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time

class Geocoding(Node):
    def __init__(self):
        super().__init__("geocoding_node")
        
        self.place_name = self.declare_parameter(
            name="place_name",
            value="Kansai University"
        ).get_parameter_value().string_value
        self.map_frame = self.declare_parameter(
            name="map_frame",
            value="map"
        ).get_parameter_value().string_value
        self.tile_map_frame = self.declare_parameter(
            name="tile_map_frame",
            value="origin"
        ).get_parameter_value().string_value
        self.node_waiting_for = self.declare_parameter(
            name="node_waiting_for",
            value="initialize_origin"
        ).get_parameter_value().string_value
        
        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer, self)
        
        tf2_futrue = self.tf2_buffer.wait_for_transform_async(self.map_frame, self.tile_map_frame, Time(seconds=0, nanoseconds=5 * 10**8))
        rclpy.spin_until_future_complete(self, tf2_futrue)
        self.get_logger().info("Transform [{} -> {}] is OK!".format(self.tile_map_frame, self.map_frame))

        while not (self.node_waiting_for in self.get_node_names()):
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1, nanoseconds=0))
            self.get_logger().info("Waiting for Node [{}] ready.....".format(self.node_waiting_for))
        self.get_logger().info("Node [{}] is OK".format(self.node_waiting_for))
        
        self.initial_geo_pub = self.create_publisher(NavSatFix, "fix", 5)
        
        for index in range(3):
            result = get_geolocation(self.get_name(), self.place_name)
            if result != None :
                msg = NavSatFix()
                msg.latitude, msg.longitude = result
                msg.header.frame_id = "wgs84"
                msg.header.stamp = self.get_clock().now().to_msg()
                self.initial_geo_pub.publish(msg)
                self.get_logger().info(
                    "-- Initialize Geolocation -- \n\t\t\t \
                        - Latitude: {} -\n\t\t\t \
                        - Longitude: {} -".format(result[0], result[1])
                )
                break
            
            if index == 2:
                self.get_logger().error("Couldn't get geo information. Please check your network and try Again!!!")
                return

def main():
    rclpy.init(args=sys.argv)
    node = Geocoding()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()