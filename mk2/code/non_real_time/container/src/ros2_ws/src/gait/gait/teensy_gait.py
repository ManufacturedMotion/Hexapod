import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import math
import time
import re
import copy

name = "teensyGait"

class TeensyGait(Node):

    def __init__(self):
        super().__init__(name)
        self.cmd_vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.parseCmdVel, 1)
        self.joy_subscriber = self.create_subscription(Joy, "/joy", self.parseJoy, 10)
        self.teensy_subscriber = self.create_subscription(String, "/from_teensy", self.parseTeensyMsg, 1)
        self.publisher = self.create_publisher(String, '/to_teensy', 10)
        self.get_logger().info(f"{name} has begun")
        self.old_joy_cmd = {}

        self.walk_scale_fact = 300
        self.yaw_scale_fact = 300
        self.z_scale_fact = 100
        self.roll_scale_fact = 100
        self.pitch_scale_fact = 100

        self.pos = {
            'x': 0.0,
            'y': 0.0,
            'z': 10.0, 
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }

        self.old_step_cmd = {
                "MV": "VSET",
                "X": 0.0,
                "Y": 0.0,
                "Z": 0.0,
                "ROLL": 0.0, 
                "PTCH": 0.0,
                "YAW": 0.0,
            }

        self.default_speed = 270
        self.speed = self.default_speed
        self.max_speed = 400
        self.drift_factor = 0.25

    def parseCmdVel(self, msg: Twist):
        
        if (abs(msg.linear.x) <= self.drift_factor and abs(msg.linear.y) <= self.drift_factor and abs(msg.linear.z) <= self.drift_factor and abs(msg.angular.x) <= self.drift_factor and abs(msg.angular.y) <= self.drift_factor and abs(msg.angular.z) <= self.drift_factor):
            step_cmd = {
                    "MV": "VSET",
                    "X": 0.0,
                    "Y": 0.0,
                    "Z": 0.0,
                    "ROLL": 0.0,
                    "PTCH": 0.0,
                    "YAW": 0.0,
                }
        else:
            self.pos['yaw'] = msg.angular.z * self.yaw_scale_fact
            self.pos['x'] = (msg.linear.y * self.walk_scale_fact)
            self.pos['y'] = (msg.linear.x * self.walk_scale_fact)

            self.pos['z'] += msg.linear.z * self.z_scale_fact
            self.pos['roll'] += msg.angular.x * self.roll_scale_fact
            self.pos['pitch'] += msg.angular.y * self.pitch_scale_fact
            
            step_cmd = {
                    "MV": "VSET",
                    "X": round(self.pos['x'], 1),
                    "Y": round(self.pos['y'], 1),
                    "Z": 0.0,
                    "ROLL": 0.0,
                    "PTCH": 0.0,
                    "YAW": round(self.pos['yaw'], 1),
                }
        if step_cmd != self.old_step_cmd:
            self.old_step_cmd = step_cmd
            command = self.prepCommand(step_cmd)
            self.publisher.publish(command)
            self.get_logger().info(f"Published command: {command.data}")

    def parseJoy(self, msg: Joy):
        
        neutral = msg.buttons[0] #A
        sit = msg.buttons[1] #B
        detach = msg.buttons[12] #Xbox button
        dance0 = msg.buttons[3] #X
        dance1 = msg.buttons[4] #Y
        dance2 = msg.buttons[6] #LB
        dance3 = msg.buttons[7] #RB
        dance4 = msg.buttons[13] #Left stick
        dance5 = msg.buttons[14] #Right stick

        #if multiple buttons pressed don't accept any input as valid
        if (sum([sit, neutral, dance0, dance1, dance2, dance3, dance4, dance5]) > 1):
            joy_cmd = {}
        elif dance0:
            joy_cmd = {"PRE": "DNC0"}
        elif dance1:
            joy_cmd = {"PRE": "DNC1"}
        elif dance2:
            joy_cmd = {"PRE": "DNC2"}
        elif dance3:
            joy_cmd = {"PRE": "DNC3"}
        elif dance4:
            joy_cmd = {"PRE": "DNC4"}
        elif dance5:
            joy_cmd = {"PRE": "DNC5"}
        elif sit:
            joy_cmd = {"PRE": "SIT"}
        elif detach:
            joy_cmd = {"PRE": "DTCH"}
        elif neutral:
            joy_cmd = {
                "MV": "RPD",
                "X": 0,
                "Y": 0,
                "Z": 200,
                "ROLL": 0,
                "PTCH": 0,
                "YAW": 0
            }
            self.pos = {
                'x': 0.0,
                'y': 0.0,
                'z': 150.0, 
                'roll': 0.0,
                'pitch': 0.0,
                'yaw': 0.0
            }
        else:
            joy_cmd = {}
        
        if joy_cmd != self.old_joy_cmd:
            self.old_joy_cmd = joy_cmd
            if joy_cmd:
                self.get_logger().info(f"msg: {msg.buttons}")
                self.get_logger().info(f"Joy command: {joy_cmd}")
                self.publisher.publish(self.prepCommand(joy_cmd))
        

    def parseTeensyMsg(self, msg):
        json_blobs = re.findall(r'\{.*?\}', msg.data)
        for json_blob in json_blobs:
            try:
                json_msg = json.loads(json_blob)
                move_time = json_msg.get("MOVE_TIME", None)
                if move_time:
                    self.send_time = (move_time / 1000) + time.time()
                    self.get_logger().info(f"updated move time: {self.send_time}")
                    self.waiting_for_ack = False
                    return
            except json.JSONDecodeError:
                self.get_logger().warn("Failed to decode portion of Json msg")
        self.send_time = 0
        self.waiting_for_ack = False

    def prepCommand(self, command):
        json_string = json.dumps(command)
        string_msg = String()
        string_msg.data = json_string
        return string_msg

def main(args=None):
    rclpy.init(args=args)
    node = TeensyGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
