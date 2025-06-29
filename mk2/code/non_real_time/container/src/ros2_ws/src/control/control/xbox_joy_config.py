# Custom node for xbox controller as default config does not allow
# both joysticks to both control linear.y
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class XboxJoyNode(Node):
    def __init__(self):
        super().__init__('xbox_joy_config')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_twist = Twist()
        self.last_joy_time = self.get_clock().now()
        self.timeout_sec = 0.5
        self.poll_rate = 20.0
        self.poll_timer = self.create_timer(1.0 / self.poll_rate, self.poll_callback)

    def joy_callback(self, msg):
        left_y = -msg.axes[0]
        left_x = -msg.axes[1]
        right_x = -msg.axes[3]
        right_y = -msg.axes[2]
        dpad_horiz = msg.axes[6]
        dpad_vert = msg.axes[7]

        threshold = 0.3

        # --- RIGHT STICK: 8-way snap, only 0, ±0.5, ±1 ---
        mag_right = math.hypot(right_y, right_x)
        discrete_x, discrete_y = 0.0, 0.0
        if mag_right > threshold:
            angle = math.atan2(right_y, right_x)
            direction = int(((math.degrees(angle) + 360 + 22.5) % 360) // 45)
            match direction:
                case 0:
                    discrete_x, discrete_y = 1.0, 0.0
                case 1:
                    discrete_x, discrete_y = 0.5, 0.5
                case 2:
                    discrete_x, discrete_y = 0.0, 1.0
                case 3:
                    discrete_x, discrete_y = -0.5, 0.5
                case 4:
                    discrete_x, discrete_y = -1.0, 0.0
                case 5:
                    discrete_x, discrete_y = -0.5, -0.5
                case 6:
                    discrete_x, discrete_y = 0.0, -1.0
                case 7:
                    discrete_x, discrete_y = 0.5, -0.5

            discrete_y *= mag_right
            discrete_x *= mag_right

        # --- LEFT STICK: Only one dominant direction, never both ---
        mag_left = math.hypot(left_y, left_x)
        lin_x, ang_z = 0.0, 0.0
        if abs(left_x) > threshold or abs(left_y) > threshold:
            if abs(left_x) > abs(left_y):
                lin_x = math.copysign(1.0, left_x)
                ang_z = 0.0
            else:
                ang_z = math.copysign(1.0, left_y)
                lin_x = 0.0
        
            ang_z *= mag_left
            lin_x *= mag_left

        twist = Twist()
        twist.linear.y = discrete_y
        twist.linear.x = discrete_x if mag_right > threshold else lin_x
        twist.angular.z = ang_z
        twist.angular.y = dpad_horiz
        twist.angular.x = dpad_vert

        self.last_twist = twist
        self.last_joy_time = self.get_clock().now()

    def poll_callback(self):
        # If timeout, publish zero Twist
        now = self.get_clock().now()
        if (now - self.last_joy_time).nanoseconds * 1e-9 > self.timeout_sec:
            zero_twist = Twist()
            self.cmd_pub.publish(zero_twist)
        else:
            self.cmd_pub.publish(self.last_twist)

def main(args=None):
    rclpy.init(args=args)
    node = XboxJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()