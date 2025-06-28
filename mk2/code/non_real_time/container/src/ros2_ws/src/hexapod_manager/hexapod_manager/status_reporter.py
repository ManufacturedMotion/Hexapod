import rclpy
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
import re
import json

name = "hexapodStatus"

class StatusReporter(Node):

    def __init__(self):
        super().__init__(name)
        self.publisher = self.create_publisher(String, "/hexapod_status", 1)   
        self.teensy_subscriber = self.create_subscription(String, "/from_teensy", self.parseTeensyMsg, 1)
        self.get_logger().info(f"{name} has begun")
        try:
            package_share = get_package_share_directory('hexapod_manager')
            json_path = os.path.join(package_share, 'resource', 'battery_table.json')
            self.get_logger().info(f"{json_path}")
            with open(json_path) as fp:
                self.battery_table = json.load(fp)
        except Exception as e:
            error_msg = String()
            error_msg.data = f"ERROR! Trouble opening battery table! {e}"
            self.publisher.publish(error_msg)
            self.battery_table = {}
        self.battery_charge = 100


    def parseTeensyMsg(self, msg):
        json_blobs = re.findall(r'\{.*?\}', msg.data)
        for json_blob in json_blobs:
            try:
                json_msg = json.loads(json_blob)
                vdd = json_msg.get("VDD", None)
                new_battery_charge = self.battery_table.get(str(vdd), "LOOKUP GAP")

                if self.battery_charge != new_battery_charge and new_battery_charge != "LOOKUP GAP":
                    self.battery_charge = new_battery_charge
                    battery_msg = String()
                    battery_msg.data = f"BATTERY CHARGE: {self.battery_charge}%"
                    self.publisher.publish(battery_msg)
            except json.JSONDecodeError:
                self.get_logger().warn("Failed to decode portion of Json msg")


def main():
    rclpy.init()
    node = StatusReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

