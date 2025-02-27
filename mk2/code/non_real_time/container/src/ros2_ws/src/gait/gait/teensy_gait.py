import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

name = "teensyGait"

class TeensyGait(Node):

    def __init__(self):
        super().__init__(name)
        self.cmd_vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.parse_cmd_vel, 10)
        self.joy_subscriber = self.create_subscription(Joy, "/joy", self.parse_joy, 10)
        self.publisher = self.create_publisher(String, '/to_teensy', 10)
        self.get_logger().info(f"{name} has begun")
        self._roll = 0
        self._pitch = 0
        self._height = 0
        self.walk_scale_fact = 10

    def parse_cmd_vel(self, msg: Twist):
        self._roll += msg.angular.x
        self._pitch += msg.angular.y
        self._height += msg.angular.z
        if (msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0):
            return
        json_cmd = {
            "MV": "WLK",
            "X": msg.linear.x * self.walk_scale_fact,
            "Y": msg.linear.y * self.walk_scale_fact,
            "Z": self._height, #not sure I consume this one right
            "ROLL": self._roll,
            "PTCH": self._pitch,
            "YAW": msg.linear.z
        }
        json_string = json.dumps(json_cmd)
        string_msg = String()
        string_msg.data = json_string
        self.publisher.publish(string_msg)
        print(f"CONSTRUCTED MSG: {json_string}")

    def parse_joy(self, msg: Joy):
        stand = msg.buttons[0]
        sit = msg.buttons[1]
        neutral = msg.buttons[3]
        zeros = msg.buttons[4]
        preset = ""
        if (sum([stand, sit, zeros, neutral]) > 1):
            return
        elif stand:
            preset = "STND" 
            self._height = 200 #? not sure if really 200
        elif sit:
            preset = "SIT"
            self._height = 0
        elif zeros:
            preset = "Z"
            self._height = 0
        elif neutral:
            self._height = 200
            json_cmd = {
                "MV": "RPD",
                "X": 100,
                "Y": 0,
                "Z": 200
            }
            json_string = json.dumps(json_cmd)
            string_msg = String()
            string_msg.data = json_string
            self.publisher.publish(string_msg)
            print("Moving to neutral position")
            return
        else:
            return
        self._roll = 0
        self._pitch = 0
        json_cmd = {
            "PRE": preset
        }
        json_string = json.dumps(json_cmd)
        string_msg = String()
        string_msg.data = json_string
        print(f"CONSTRUCTED PRESET: {json_cmd}")
        self.publisher.publish(string_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeensyGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
