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
        self.teensy_subscriber = self.create_subscription(String, "/from_teensy", self.parseTeensyMsg, 1)
        self.publisher = self.create_publisher(String, '/to_teensy', 10)
        self.publish_timer = self.create_timer(0.2, self.updateCommandList) 
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
        self.walk_scale_fact = 4
        self.default_speed = 250
        self.speed = self.default_speed
        self.max_speed = 400
        self.drift_factor = 0.1
        self.cmd_vel_received = False
        self.minimum_step_size = 100
        self.maximum_step_size = 180
        self.last_send_time = 0
        self.send_time = 0
        self.last_cmd = {}
        self.cmd_timeout = 0.5
        self.waiting_for_ack = False

    def updateCommandList(self):

        current_time = time.time()
         
        step_cmd = None
        if self.cmd_vel_received:
            distance = self.getDistance() 
            self.setSpeed(distance)
            step_cmd = {
                "MV": "WLK",
                "X": round(self.pos['x'], 3),
                "Y": round(self.pos['y'], 3),
                "Z": round(self.pos['z'], 3),
                "ROLL": round(self.pos['roll'], 3),
                "PTCH": round(self.pos['pitch'], 3),
                "YAW": round(self.pos['yaw'], 3),
                "SPD": self.speed
            }
        else:
            distance = 0       
 
        #state for startup - logic for sending first command or command after idle period
        if self.send_time == 0:
            if distance >= self.minimum_step_size:
                prepped_cmd = self.prepCommand(step_cmd)
                self.sendCommand(prepped_cmd)
            elif (self.last_send_time + self.cmd_timeout) >= current_time:
                if step_cmd:
                    prepped_cmd = self.prepCommand(step_cmd)
                    self.sendCommand(prepped_cmd)
            elif self.joy_cmd:
                prepped_cmd = self.prepCommand(self.joy_cmd)
                if prepped_cmd != self.last_cmd:
                    self.sendCommand(prepped_cmd)
                self.joy_cmd = {}
                    
        #if we have a time acknowledgement from teensy we wait until that time before we publish the next command
        #when is it time to send a command we check if multiple inputs were received. If a joy command is recieved, we immediately send the joy (no optimization for joy msgs)
        #if the joy command is received while we are optimizing a step command we will send the joy and forget about the partial step
        elif self.send_time >= current_time:

            if self.joy_cmd:
                prepped_cmd = self.prepCommand(self.joy_cmd)
                if prepped_cmd != self.last_cmd:
                    self.sendCommand(prepped_cmd)
                self.joy_cmd = {}
      
            elif step_cmd:
                prepped_cmd = self.prepCommand(step_cmd)
                self.sendCommand(prepped_cmd)
        
        #prevent getting stuck.if we send a cmd and then go idle we reset the send time to 0
        elif current_time >= self.send_time + self.cmd_timeout:
            self.send_time = 0
                

    def parseCmdVel(self, msg: Twist):

        #no update if joystick within drift threshold
        if (abs(msg.linear.x) <= self.drift_factor and abs(msg.linear.y) <= self.drift_factor and abs(msg.linear.z) <= self.drift_factor and abs(msg.angular.x) <= self.drift_factor and abs(msg.angular.y) <= self.drift_factor and abs(msg.angular.z) <= self.drift_factor):
            return

        #if we hit max distance do not update x, y or yaw
        distance = self.getDistance()
        if distance < self.maximum_step_size:
            self.pos['yaw'] += msg.angular.z
            self.pos['x'] += (msg.linear.y * self.walk_scale_fact)
            self.pos['y'] += (msg.linear.x * self.walk_scale_fact)
 
        self.pos['roll'] += msg.angular.x
        self.pos['pitch'] += msg.angular.y
        #TODO need to update z somehow
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
                self.send_time = (move_time / 1000) + time.time()
        except json.JSONDecodeError:
            self.get_logger().warn("Failed to decode Json")
            self.send_time = 0
        self.waiting_for_ack = False

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
        self.speed = self.default_speed
    
    def getDistance(self):
        x = self.pos['x']
        y = self.pos['y']
        yaw = self.pos['yaw']
        distance = math.sqrt(x**2 + y**2 + (yaw*100)**2)
        return distance 

    def setSpeed(self, distance):
        if distance <= self.minimum_step_size:
            self.speed = self.default_speed
        #if we are above minimum step size we can scale the speed up a bit
        else:
            sensitivity = 0.02
            scaled_speed = self.default_speed + (math.exp(distance * sensitivity) - 1)
            self.speed = round(min(scaled_speed, self.max_speed), 2)
        
    def sendCommand(self, command):
        if self.waiting_for_ack:
            return
        self.publisher.publish(command)
        self.waiting_for_ack = True
        self.last_cmd = command
        self.resetPos()
        self.joy_cmd = {}
        self.cmd_vel_received = False
        self.last_send_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = TeensyGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
