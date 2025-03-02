import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import math
import time
import copy

name = "teensyGait"

class TeensyGait(Node):

    def __init__(self):
        super().__init__(name)
        self.cmd_vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.parse_cmd_vel, 10)
        self.joy_subscriber = self.create_subscription(Joy, "/joy", self.parse_joy, 10)
        self.publisher = self.create_publisher(String, '/to_teensy', 10)
        self.publish_timer = self.create_timer(0.1, self.publishCommand) #publish command every .2 seconds       
        self.get_logger().info(f"{name} has begun")
        self.joy_cmd = {}
        self.last_joy_cmd = {}
        self.pos = {
            "x": 0,
            "y": 0,
            "z": 0,
            "roll": 0,
            "pitch": 0,
            "yaw": 0
        }
        self.step = {
            "x": 0,
            "y": 0
        }
        self.walk_scale_fact = 4
        self.minimum_step_size = 30
        self.last_command_time = time.time()
        self.step_timeout = 1.5
        self.drift_factor = 0.1
        self.need_to_move = False

    def publishCommand(self):

        current_time = time.time()
        time_since_last_publish = current_time - self.last_command_time

        if self.joy_cmd and self.joy_cmd != self.last_joy_cmd:
            if "MV" in self.joy_cmd.keys() or self.joy_cmd["PRE"] == "STND":
                self.resetPos(200) #TODO measure z height for stand, if not 200 make new logic here
            else:
                self.resetPos()
            self.publisher.publish(self.prepCommand(self.joy_cmd))
            self.last_joy_cmd = copy.deepcopy(self.joy_cmd)
            self.joy_cmd = {}
            self.last_command_time = current_time
            
        elif self.getStepDistance() >= self.minimum_step_size or (time_since_last_publish >= self.step_timeout and self.need_to_move):
            step_command = {
                "MV": "WLK",
                "X": round(self.step['x'], 3),
                "Y": round(self.step['y'], 3),
                "Z": round(self.pos['z'], 3),
                "ROLL": round(self.pos['roll'], 3),
                "PTCH": round(self.pos['pitch'], 3),
                "YAW": round(self.pos['yaw'], 3)
            }
            self.publisher.publish(self.prepCommand(step_command))
            self.pos['x'] += self.step['x']
            self.pos['y'] += self.step['y']
            self.step['x'] = 0
            self.step['y'] = 0
            self.last_command_time = current_time
            self.need_to_move = False
            self.last_joy_cmd = {}

    def parse_cmd_vel(self, msg: Twist):

        if (abs(msg.linear.x) <= self.drift_factor and abs(msg.linear.y) <= self.drift_factor and abs(msg.linear.z) <= self.drift_factor and abs(msg.angular.x) <= self.drift_factor and abs(msg.angular.y) <= self.drift_factor and abs(msg.angular.z) <= self.drift_factor):
            return
        
        self.pos['roll'] += msg.angular.x
        self.pos['pitch'] += msg.angular.y
        self.pos['yaw'] += msg.angular.z
        #walk scale will eventually move to Danny's twist yaml 
        self.step['x'] += (msg.linear.x * self.walk_scale_fact)
        self.step['y'] += (msg.linear.y * self.walk_scale_fact)
        self.need_to_move = True

    def parse_joy(self, msg: Joy):
        stand = msg.buttons[0]
        sit = msg.buttons[1]
        neutral = msg.buttons[3]
        zeros = msg.buttons[4]
        if (sum([stand, sit, zeros, neutral]) > 1):
            return
        else:
            self.joy = {}

        if stand:
            self.joy_cmd = {"PRE": "STND"}
        elif sit:
            self.joy_cmd = {"PRE": "SIT"}
        elif zeros:
            self.joy_cmd = {"PRE": "Z"}
        elif neutral:
            self.joy_cmd = {
                "MV": "RPD",
                "X": 0,
                "Y": 0,
                "Z": 200
            }
        else:
            return

        if self.joy_cmd == self.last_joy_cmd:
            self.joy_cmd = {}

    def prepCommand(self, command):
        json_string = json.dumps(command)
        string_msg = String()
        string_msg.data = json_string
        print(f"CONSTRUCTED COMMAND: {command}")
        return string_msg
    
    def getStepDistance(self):
        dx = self.step['x']
        #dx = self.pos['x'] - self.step['x']
        dy = self.step['y'] 
        #dy = self.pos['y'] - self.step['y'] 
        distance = math.sqrt( dx ** 2 + dy ** 2)
        return distance

    def resetPos(self, z = 0):
        yaw = self.pos['yaw']
        self.pos = {
            "x": 0,
            "y": 0,
            "z": z,
            "roll": 0,
            "pitch": 0,
            "yaw": yaw
        }
        self.step = {
            "x": 0,
            "y": 0
        }

def main(args=None):
    rclpy.init(args=args)
    node = TeensyGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
