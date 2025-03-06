from rclpy.node import Node
import rclpy, os, subprocess
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import String
import xml.etree.ElementTree as ET

class GoogleEarthNode(Node):
    def __init__(self):
        super().__init__("google_earth_node")
        
        self.pub_geo_polygon = self.create_publisher(PolygonStamped, "topological_map", 1)
        self.pub_kml_path = self.create_publisher(String, "kml_file_path", 1)
        self.main_timer = self.create_timer(0.5, self.wait_for_kml)
        
        self.google_earth_process = subprocess.Popen("google-chrome --new-window https://earth.google.com/web", stdout=subprocess.PIPE, shell=True, text=True)
    
    def wait_for_kml(self):
        # print(self.google_earth_process.pid)
        # print(self.google_earth_process.terminate())
        base_dir = os.path.join(os.environ["HOME"], "Downloads")
        kml_url_list = []
        for dirpath, dirnames, filenames in os.walk(base_dir):
            for filename in filenames:
                if ".kml" in filename:
                    kml_url_list.append(os.path.join(base_dir, filename))
        
        if len(kml_url_list) != 0:
            for kml_url in kml_url_list:
                tree = ET.parse(kml_url)
                root = tree.getroot()
                coordinates = root.find('.//kml:coordinates', {'kml': 'http://www.opengis.net/kml/2.2'})

                raw_coords = coordinates.text.strip()
                polygon = PolygonStamped()
                polygon.header.frame_id = "wgs84"
                polygon.header.stamp = self.get_clock().now().to_msg()
                points = []
                for coord in raw_coords.split():
                    point = Point32()
                    lon, lat, alt = map(float, coord.split(','))
                    point.x = lon
                    point.y = lat
                    point.z = alt
                    points.append(point)
                
                polygon.polygon.points = points
                self.pub_geo_polygon.publish(polygon)
                
                kml_path = String()
                kml_path.data = kml_url
                self.pub_kml_path.publish(kml_path)
                    

def main():
    rclpy.init()
    Ge_node = GoogleEarthNode()
    rclpy.spin(Ge_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
