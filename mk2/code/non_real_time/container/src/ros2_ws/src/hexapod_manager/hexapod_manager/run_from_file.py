import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys

name = "RunFromFile"

class RunFromFile(Node):

    def __init__(self):
        super().__init__(name)
        self.publisher = self.create_publisher(String, '/to_teensy', 10)

    def publish_line(self, line):
        msg = String()
        msg.data = line
        self.publisher.publish(msg)
        print(f'Published: "{line}"')

def main(args=None):

    if len(sys.argv) != 2:
        print("ERROR! Must run this node with a text file containing commands! Ex: ros2 run hexapod_manager run_from_file.py <txt_file>")
        return

    file_path = sys.argv[1]

    rclpy.init(args=args)
    node = RunFromFile()

    try:
        with open(file_path, 'r') as file:
            for line in file:
                clean_line = line.strip()
                if clean_line: 
                    if "delay" in clean_line or "sleep" in clean_line:
                        delay_time = clean_line.split(" ")[-1]
                        time.sleep(int(delay_time))
                        continue
                    clean_line += "\n"
                    node.publish_line(clean_line)
                    time.sleep(1) #TODO - optimize time between sending messages?
    except Exception as e:
        print(f"Failed to read file: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
