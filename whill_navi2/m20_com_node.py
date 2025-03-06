import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import requests, json
import os
import sys

class M20COM(Node):
    
    def __init__(self):
        super().__init__("m20_com_node")
        
        ip_address = self.declare_parameter("ip_address", "192.168.0.10").get_parameter_value().string_value
        self.mode = self.declare_parameter("mode", "manual").get_parameter_value().string_value
        self.m20_lidar_node_name = self.declare_parameter("m20_lidar_node_name", "lumotive_driver").get_parameter_value().string_value
        self.angle_range = self.declare_parameter("angle_range", [-45, 45]).get_parameter_value().integer_array_value
        ip_address = "http://" + ip_address
        self.state = ""
        self.initialize_ok = False

        self.post_command_dict = {
            "start_scan" : ip_address + "/start_scan",
            "stop_scan" : ip_address + "/stop_scan",
            "angle_range": ip_address + "/angle_range",
            "scan_parameters": ip_address + "/scan_parameters"
        }
        
        self.get_command_dict = {
            "get_state" : ip_address + "/state",
            "scan_parameters" : ip_address + "/scan_parameters"
        }
        
        self.headers = {"Content-Type" : "application/json"}
                        
    def is_online(self) -> bool:        
        for index in range(6):
            try:
                state_response = requests.get(url=self.get_command_dict["get_state"], headers=self.headers, timeout=0.5)
                self.state = state_response.json()["state"]
            except requests.exceptions.ConnectTimeout as e:
                self.get_logger().warn("Socket Error: {}".format(e))
                return False
            
            if state_response.status_code == 200:
                self.get_logger().info("M20_Lidar is online", once=True)
                return True
            else:
                self.get_logger().warn("Trying to connect to M20_Lidar...")
                self.get_clock().sleep_for(Duration(seconds=0, nanoseconds=int(5e8)))
        
        return False
        
    def auto_control(self):
        
        if self.m20_lidar_node_name in self.get_node_names():
            if self.state == "ENERGIZED":
                # Start scan
                requests.post(url=self.post_command_dict["start_scan"], headers=self.headers, timeout=5.0)                
                self.get_logger().info("Change state into SCANNING")
                
            elif self.state == "SCANNING":
                # Get scan parameters
                get_scan_param_res = requests.get(url=self.get_command_dict["scan_parameters"], headers=self.headers, timeout=5.0)
                scan_param_json = get_scan_param_res.json()
                if self.angle_range[0] < -45:
                    self.get_logger().warn("Minimum angle range is -45, but {} given. Set the minimum angle range to -45".format(self.angle_range[0]))
                    self.angle_range[0] = -45
                if self.angle_range[1] > 45:
                    self.get_logger().warn("Maximum angle range is 45, but {} given. Set the minimum angle range to 45".format(self.angle_range[1]))
                    self.angle_range[1] = 45
                scan_param_json["angle_range"] = list(self.angle_range)
                
                # Set scan parameters
                post_scan_param_res = requests.post(url=self.post_command_dict["scan_parameters"], headers=self.headers, json=scan_param_json, timeout=2.0)
                self.get_logger().info("Change angle range into [{} ~ {}]!".format(self.angle_range[0], self.angle_range[1]), once=True)
                self.initialize_ok = True
                                    
    def __del__(self):
        if self.state == "SCANNING" and self.mode != "manual":
            response = requests.post(url=self.post_command_dict["stop_scan"], headers=self.headers, timeout=1.0)
                
def main():
    rclpy.init(args=sys.argv)
    os.environ.pop("http_proxy", None)
    os.environ.pop("https_proxy", None)
    os.environ.pop("HTTP_PROXY", None)
    os.environ.pop("HTTPS_PROXY", None)

    node = M20COM()
    input_argv = sys.argv
    if "-d" in input_argv:
        index = input_argv.index("-d")
        input_argv.pop(index)
        url = input_argv.pop(index)
    else:
        url = "192.168.0.10"
    
    if node.mode == "manual":
        if len(input_argv) < 3:
            node.get_logger().warn("m20_com_node usage: [GET or POST] [-d URL (optional)] [ENDPOINT]")
            return

        action = input_argv[1]
        endpoint = input_argv[2]
        
        if len(input_argv) == 4:
            send_data = input_argv[3]
        else:
            send_data = None
        
        if action == "GET":
            response = requests.get(url="http://" + url + "/{}".format(endpoint), headers=node.headers)
            node.get_logger().info(response.text)
        elif action == "POST":
            if send_data:
                try:
                    json_data = json.loads(send_data)
                except Exception as e:
                    node.get_logger().error("Get json exception: {}".format(e))
                    rclpy.shutdown()
                    return
            else:
                json_data = None
            
            response = requests.post(url="http://" + url + "/{}".format(endpoint), headers=node.headers, json=json_data)
            node.get_logger().info(response.text)
        else:
            node.get_logger().error("Please give me a command which is 'GET' or 'POST'.")
            
        node.destroy_node()
        rclpy.shutdown()
        return

    elif node.mode == "auto":
        while rclpy.ok():
            if not node.is_online():
                node.get_logger().warn("Waitting for the sensor ready...")
                node.get_clock().sleep_for(Duration(seconds=0, nanoseconds=int(5e8)))
                continue
            
            if not node.initialize_ok:
                node.auto_control()
            
            node.get_clock().sleep_for(Duration(seconds=1, nanoseconds=0))
            
    else:
        node.get_logger().error("Get a wrong mode which is {}, please use 'manual' or 'auto' to set this parameter".format(node.mode))
        rclpy.shutdown()
        
        # print(node.angle_range)
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()