import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import board
import neopixel_spi as neopixel
import threading

LED_COUNT = 60
LED_PIN = board.SPI()
DEBOUNCE_DELAY = 0.3

# Colors
off = (0, 0, 0, 0)
colors = {
    "pink": (245, 0, 87, 0), "red": (255, 0, 0, 0), "green": (0, 255, 0, 0), "blue": (0, 0, 255, 0),
    "white": (0, 0, 0, 255), "purple": (106, 27, 154, 0), "lavender": (234, 128, 252, 0),
    "light_blue": (0, 176, 255, 0), "cyan": (0, 172, 193, 0), "mint": (100, 255, 218, 0),
    "lime": (118, 255, 3, 0), "yellow": (255, 255, 0, 0), "orange": (230, 81, 0, 0)
}

class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller')
        self.pixels = neopixel.NeoPixel_SPI(LED_PIN, LED_COUNT, pixel_order=neopixel.GRBW, auto_write=True)

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.pattern_index = 0
        self.running = True
        self.last_button_press = 0
        self.animation_thread = threading.Thread(target=self.run_animations, daemon=True)
        self.animation_thread.start()

        self.get_logger().info("LED Controller Node Started")

    def clear_pixels(self):
        self.pixels.fill(off)

    def loop(self, color=colors["red"], width=1, delay=0.005, reverse=False):
        for _ in range(3):
            for pixel in range(LED_COUNT):
                if not self.running:
                    return
                time.sleep(delay)
                index = LED_COUNT - 1 - pixel if reverse else pixel
                off_index = index + 1 if reverse else index - 1
                self.pixels[off_index % LED_COUNT] = off
                for i in range(width):
                    self.pixels[(index - i) % LED_COUNT if reverse else (index + i) % LED_COUNT] = color
        self.clear_pixels()

    def windmill(self, color_1=colors["orange"], color_2=colors["purple"], delay=0.05, sections=4, reverse=False):
        for _ in range(2):
            for pixel in range(LED_COUNT):
                if not self.running:
                    return
                time.sleep(delay)
                index = LED_COUNT - 1 - pixel if reverse else pixel
                for section in range(sections):
                    color = color_1 if section % 2 == 0 else color_2
                    self.pixels[(index + section * (LED_COUNT // sections)) % LED_COUNT] = color
        self.clear_pixels()

    def pattern(self, color=colors["red"], width=1, delay=0.2):
        pixel_group = [i for i in range(0, LED_COUNT, 2 * width)]
        for _ in range(4):
            if not self.running:
                return
            time.sleep(delay)
            self.pixels.fill(color)
            for pixel in pixel_group:
                self.pixels[pixel] = off
            time.sleep(delay)
            self.pixels.fill(color)
            for pixel in range(LED_COUNT):
                if pixel not in pixel_group:
                    self.pixels[pixel] = off
        self.clear_pixels()

    def joy_callback(self, msg):
        current_time = time.time()
        
        if len(msg.buttons) > 10 and msg.buttons[10] == 1:
            if current_time - self.last_button_press < DEBOUNCE_DELAY:
                return
            
            self.last_button_press = current_time
            self.pattern_index = (self.pattern_index + 1) % 3
            
            self.running = False
            time.sleep(0.1)
            self.running = True
            self.animation_thread = threading.Thread(target=self.run_animations, daemon=True)
            self.animation_thread.start()

    def run_animations(self):
        while self.running:
            if self.pattern_index == 0:
                self.loop()
            elif self.pattern_index == 1:
                self.windmill(color_1=colors["mint"], color_2=colors["red"])
            elif self.pattern_index == 2:
                self.pattern()

    def stop(self):
        self.running = False
        self.animation_thread.join()
        self.clear_pixels()
        self.get_logger().info("LED Controller Shutting Down")


def main():
    rclpy.init()
    node = LEDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
