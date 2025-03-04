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

off = (0, 0, 0, 0)
colors = {
    "red": (255, 0, 0, 0), "green": (0, 255, 0, 0), "blue": (0, 0, 255, 0),
    "white": (0, 0, 0, 255), "purple": (106, 27, 154, 0), "yellow": (255, 255, 0, 0),
}
color_keys = list(colors.keys())

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
        self.color_index = 0
        self.color_1 = color_keys[self.color_index]
        self.animation_thread = threading.Thread(target=self.run_animations, daemon=True)
        self.animation_thread.start()

    def clear_pixels(self):
        self.pixels.fill(off)

    def loop(self, color, delay=0.005, reverse=False):
        color_val = colors.get(color, off)
        width = 3
        
        for _ in range(LED_COUNT):
            if not self.running:
                return
            
            for i in range(width):
                index = (-_ - i) % LED_COUNT if reverse else (_ + i) % LED_COUNT
                self.pixels[index] = color_val

            time.sleep(delay)

            tail_index = (-_ - width) % LED_COUNT if reverse else (_ - 1) % LED_COUNT
            self.pixels[tail_index] = off

    def windmill(self, color, delay=0.05, sections=4, reverse=False):
        primary_color = colors.get(color, off)
        secondary_color = colors["white"]

        for _ in range(1):
            for pixel in range(LED_COUNT):
                if not self.running:
                    return
                time.sleep(delay)

                index = LED_COUNT - 1 - pixel if reverse else pixel

                for section in range(sections):
                    section_color = primary_color if section % 2 == 0 else secondary_color
                    section_index = (index + section * (LED_COUNT // sections)) % LED_COUNT
                    self.pixels[section_index] = section_color

    def pattern(self, color, width=1, delay=0.2):
        color_val = colors.get(color, off)
        pixel_group = [i for i in range(0, LED_COUNT, 2 * width)]
        for _ in range(4):
            if not self.running:
                return
            time.sleep(delay)
            self.pixels.fill(color_val)
            for pixel in pixel_group:
                self.pixels[pixel] = off
            time.sleep(delay)
            self.pixels.fill(color_val)
            for pixel in range(LED_COUNT):
                if pixel not in pixel_group:
                    self.pixels[pixel] = off

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
        
        if len(msg.buttons) > 11 and msg.buttons[11] == 1:
            if current_time - self.last_button_press < DEBOUNCE_DELAY:
                return
            
            self.last_button_press = current_time
            self.color_index = (self.color_index + 1) % len(color_keys)
            self.color_1 = color_keys[self.color_index]

    def run_animations(self):
        while self.running:
            if self.pattern_index == 0:
                self.loop(self.color_1)
            elif self.pattern_index == 1:
                self.windmill(self.color_1)
            elif self.pattern_index == 2:
                self.pattern(self.color_1)

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
