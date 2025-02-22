import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial

name = "TeensyNode"
teensy_port = "/dev/ttyS0" # TODO: Confirm
teensy_baudrate = 115200
serial_checking_period = 0.01

class TeensyNode(Node):
    def __init__(self):
        super().__init__(name)
        self._cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.on_cmd_vel_msg, 10
        )
        self.ser = serial.Serial(teensy_port, teensy_baudrate, timeout=1)
        self.publisher_ = self.create_publisher(String, '/teensy_receive', 10)
        self.subscriber = self.create_subscription(String,'teensy_send',self.send_to_teensy,10)
        self.timer = self.create_timer(serial_checking_period, self.get_and_circulate_responses)
        self.get_logger().info(f'{name} has begun')

    def send_to_teensy(self, msg):
        self.ser.write(msg.data.encode('UTF-8'))
        self.get_logger().info(f'{name} wrote {msg.data} to teensy')

    def get_and_circulate_responses(self):
        if self.ser.in_waiting > 0:
            response = self.ser.readline().decode().strip()
            msg = String()
            msg.data = response
            self.publisher.publish(msg)
            self.get_logger().info(f"{name} received {msg.data} from teensy")

    def on_cmd_vel_msg(self, msg: Twist):
        self.get_logger().info(f"Got msg: {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = TeensyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()