# Custom node for xbox controller as default config does not allow
# both joysticks to both control linear.y

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyPriorityNode(Node):
    def __init__(self):
        super().__init__('joy_priority_node')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def joy_callback(self, msg):
        # Axes mapping for Xbox controller:
        # Left stick:  vertical=1, horizontal=0
        # Right stick: vertical=3, horizontal=2
        # D-Pad: horizontal=6, vertical=7

        left_y = msg.axes[1]
        right_y = msg.axes[3]
        left_x = msg.axes[0]
        right_x = msg.axes[2]
        dpad_horiz = msg.axes[6]
        dpad_vert = msg.axes[7]

        # Handling for if each stick is pressed. Favor right joystick
        if abs(right_y) >= abs(left_y):
            linear_y = right_y
        else:
            linear_y = left_y

        twist = Twist()
        twist.linear.y = linear_y
        twist.linear.x = -msg.axes[2]  # Right stick horizontal, scale -1.0
        twist.angular.z = msg.axes[0]  # Left stick horizontal, scale 1.0

        # D-Pad mappings
        twist.angular.y = dpad_horiz   # D-Pad horizontal (pitch)
        twist.angular.x = dpad_vert    # D-Pad vertical (roll)

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyPriorityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()