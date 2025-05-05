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
        self.cmd_vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.parseCmdVel, 10)
        self.joy_subscriber = self.create_subscription(Joy, "/joy", self.parseJoy, 10)
        self.teensy_subscriber = self.create_subscription(String, "/from_teensy", self.parseTeensyMsg, 10)
        self.publisher = self.create_publisher(String, '/to_teensy', 10)
        self.publish_timer = self.create_timer(0.1, self.updateCommandList) 
        self.get_logger().info(f"{name} has begun")
        self.joy_cmd = {}
        self.pos = {
            "x": 0,
            "y": 0,
            "z": 0,
            "roll": 0,
            "pitch": 0,
            "yaw": 0
        }
        self.walk_scale_fact = 10
        self.default_speed = 150
        self.speed = self.default_speed
        self.speed_scale_fact = 1
        self.command_list = []
        self.last_append_time = 0
        self.step_timeout = 1.2
        self.drift_factor = 0.1
        self.cmd_vel_received = False
        self.send_time = 0

    def updateCommandList(self):

        current_time = time.time()
        time_since_last_append = current_time - self.last_append_time
        
        #if we have a time acknowledgement from teensy we wait until that time before we publish the next command
        if self.send_time >= current_time:
            command = self.command_list.pop(0)
            self.publisher.publish(command)
        #if we have not yet sent a message we check for the first message in our list. If we have it we send it
        elif self.send_time == 0:
            if len(self.command_list) > 0:
                command = self.command_list.pop(0)
                self.publisher.publish(command)
        #otherwise update the command queue
        #if timeout has not yet been hit, but a joy command is recieved, we immediately send the joy (no optimization for joy msgs)
        #if the joy command is received while we are optimizing a step command we will send the current step and then the joy command
        else:
            step_command = None
            if self.cmd_vel_received:
                step_command = {
                    "MV": "WLK",
                    "X": round(self.pos['x'], 3),
                    "Y": round(self.pos['y'], 3),
                    "Z": round(self.pos['z'], 3),
                    "ROLL": round(self.pos['roll'], 3),
                    "PTCH": round(self.pos['pitch'], 3),
                    "YAW": round(self.pos['yaw'], 3),
                    "SPD": self.speed
                }

            # If both are ready, send the walk step first 
            if step_command and self.joy_cmd:
                self.command_list.append(self.prepCommand(step_command))
                self.resetPos()
                self.cmd_vel_received = False
                
                #make sure we don't send the same joy cmd back to back. It wouldn't do anything
                prepped_joy_cmd = self.prepCommand(self.joy_cmd)
                if not self.command_list or self.command_list[-1] != prepped_joy_cmd:
                    self.command_list.append(prepped_joy_cmd)

                self.joy_cmd = {}
                self.last_append_time = current_time

            # if we started building a step and timeout was reached
            elif step_command and (time_since_last_append >= self.step_timeout):
                self.command_list.append(self.prepCommand(step_command))
                self.resetPos()
                self.last_append_time = current_time
                self.cmd_vel_received = False

            elif self.joy_cmd:
                prepped_joy_cmd = self.prepCommand(self.joy_cmd)
                if not self.command_list or self.command_list[-1] != prepped_joy_cmd:
                    self.command_list.append(prepped_joy_cmd)
                self.joy_cmd = {}
                self.last_append_time = current_time

    def parseCmdVel(self, msg: Twist):

        #no update if joystick within drift threshold
        if (abs(msg.linear.x) <= self.drift_factor and abs(msg.linear.y) <= self.drift_factor and abs(msg.linear.z) <= self.drift_factor and abs(msg.angular.x) <= self.drift_factor and abs(msg.angular.y) <= self.drift_factor and abs(msg.angular.z) <= self.drift_factor):
            return
        
        self.pos['roll'] += msg.angular.x
        self.pos['pitch'] += msg.angular.y
        self.pos['yaw'] += msg.angular.z
        self.pos['x'] += (msg.linear.y * self.walk_scale_fact)
        self.pos['y'] += (msg.linear.x * self.walk_scale_fact)
        #$TODO - z value. need an input for height changes? 
        self.speed = self.getSpeed(msg.linear.x, msg.linear.y)
        self.cmd_vel_received = True

    def parseJoy(self, msg: Joy):
        stand = msg.buttons[0]
        sit = msg.buttons[1]
        neutral = msg.buttons[3]
        zeros = msg.buttons[4]
        #if multiple buttons pressed don't accept any input as valid
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
                "Z": 150
            }
        else:
            return

    def parseTeensyMsg(self, msg):
        try:
            json_msg = json.loads(msg.data)
            move_time = json_msg.get("MOVE_TIME", None)
            if move_time:
                self.send_time = move_time + time.time()
                self.get_logger().info(f"Movement time is {move_time} ms")
        except json.JSONDecodeError:
            self.get_logger().warn("Failed to decode Json")

    def prepCommand(self, command):
        json_string = json.dumps(command)
        string_msg = String()
        string_msg.data = json_string
        print(f"CONSTRUCTED COMMAND: {command}")
        return string_msg
    
    def resetPos(self, z = 0):
        self.pos = {
            "x": 0,
            "y": 0,
            "z": 0,
            "roll": 0,
            "pitch": 0,
            "yaw": 0
        }
    
    #TODO - this can be improved. do we need to account for roll, pitch, yaw, z?
    def getSpeed(self, x, y):
        distance = math.sqrt( x ** 2 + y ** 2)
        speed = ((distance+1)*self.default_speed*self.speed_scale_fact)
        return round(speed, 2)

def main(args=None):
    rclpy.init(args=args)
    node = TeensyGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
