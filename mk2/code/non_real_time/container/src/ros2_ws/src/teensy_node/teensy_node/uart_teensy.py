import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial

name = "TeensyNode"
teensy_port = "/dev/ttyAMA0" 
teensy_baudrate = 250000
serial_checking_period = 0.1

class TeensyNode(Node):

    def __init__(self):
        super().__init__(name)
        self.subscriber = self.create_subscription(String, "/to_teensy", self.sendSerial, 5)
        self.serial = serial.Serial(teensy_port, teensy_baudrate, timeout=1)
        self.publisher = self.create_publisher(String, '/from_teensy', 10)
        self.timer = self.create_timer(serial_checking_period, self.circulateResponses)
        self.msg_send_queue = []
        self.get_logger().info(f'{name} has begun')

    def sendSerial(self, msg):
        self.serial.write(msg.data.encode('UTF-8') + b'\n')
        self.get_logger().info(f'{name} writing to teensy serial: {msg.data}')

    def circulateResponses(self):
        if self.serial.in_waiting > 0:
            response = self.serial.readline().decode().strip()
            msg = String()
            msg.data = response
            self.publisher.publish(msg)
            self.get_logger().info(f"{name} received {msg.data} from teensy")

def main(args=None):
    rclpy.init(args=args)
    node = TeensyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
