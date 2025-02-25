import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
import json

#TODO: define ros msg for hexapod position 

name = "GaitNode"
gait_run_period = 0.1

class GaitNode(Node):
    def __init__(self):
        super().__init__(name)
        self.publisher_ = self.create_publisher(String, '/teensy_receive', 10)
        self.command_sub = self.create_subscription(Twist,'/gait_receive',self.process_command,10)
        self.teensy_sub = self.create_subscription(String,'/teensy_send',self.process_feedback,10)
        self.timer = self.create_timer(gait_run_period, self.send_new_motion)
        self.get_logger().info(f'{name} has begun')
        self.current_command = Twist()
        self.current_leg_positions = [Point() for _ in range(6)]

    def process_command(self, msg):
        self.current_command = msg

    def process_feedback(self, msg):
        self.current_leg_positions = json.load(msg.data)

    def send_new_motion(self):
        teensy_command = 'G8 '
        for leg in range(6):
            teensy_command += f'L{leg} '
            teensy_command += f'X{self.current_leg_positions[leg].x + self.current_command.linear.x * gait_run_period} '
            teensy_command += f'Y{self.current_leg_positions[leg].y + self.current_command.linear.y * gait_run_period} '
            teensy_command += f'Z{self.current_leg_positions[leg].z + self.current_command.linear.z * gait_run_period} '
            # Remove the below once active feedback from the teensy is available
            self.current_leg_positions[leg].x += self.current_command.linear.x * gait_run_period
            self.current_leg_positions[leg].y += self.current_command.linear.y * gait_run_period
            self.current_leg_positions[leg].z += self.current_command.linear.z * gait_run_period

def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()