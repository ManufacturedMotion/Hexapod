import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import board
import neopixel_spi as neopixel
import threading
import math

LED_COUNT = 60
LED_PIN = board.SPI()
DEBOUNCE_DELAY = 0.5
DEAD_ZONE = 0.1

OFF = (0, 0, 0, 0)
COLORS = {
    "red": (255, 0, 0, 0), "green": (0, 255, 0, 0), "blue": (0, 0, 255, 0),
    "white": (0, 0, 0, 255), "purple": (106, 27, 154, 0), "yellow": (255, 255, 0, 0),
}
COLOR_KEYS = list(COLORS.keys())


class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller')
        self.pixels = neopixel.NeoPixel_SPI(LED_PIN, LED_COUNT, pixel_order=neopixel.GRBW, auto_write=True)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pattern_index = 0
        self.color_index = 0
        self.current_color = COLOR_KEYS[self.color_index]
        self.last_x = self.last_y = 0
        self.last_button_press_time = 0
        self.is_running = False
        self.animation_thread = None
        self.start_animation()

    def clear_leds(self):
        self.pixels.fill(OFF)

    def joystick_direction_effect(self, x_axis, y_axis):
        if abs(x_axis) < DEAD_ZONE and abs(y_axis) < DEAD_ZONE:
            self.clear_leds()
            return
        
        x_axis = -x_axis  
        angle = (math.degrees(math.atan2(y_axis, x_axis)) + 360) % 360
        led_center = round((angle / 360) * LED_COUNT) % LED_COUNT

        if abs(x_axis - self.last_x) < 0.05 and abs(y_axis - self.last_y) < 0.05:
            return
        
        self.last_x, self.last_y = x_axis, y_axis
        self.clear_leds()
        for i in range(-2, 3):
            self.pixels[(led_center + i) % LED_COUNT] = COLORS.get(self.current_color, OFF)

    def joy_callback(self, msg):
        current_time = time.time()
        
        if len(msg.buttons) > 10 and msg.buttons[10] == 1 and current_time - self.last_button_press_time >= DEBOUNCE_DELAY:
            self.last_button_press_time = current_time
            self.pattern_index = (self.pattern_index + 1) % 5
            self.start_animation()

        if len(msg.buttons) > 11 and msg.buttons[11] == 1 and current_time - self.last_button_press_time >= DEBOUNCE_DELAY:
            self.last_button_press_time = current_time
            self.color_index = (self.color_index + 1) % len(COLOR_KEYS)
            self.current_color = COLOR_KEYS[self.color_index]
            self.start_animation()
        
        if self.pattern_index == 4:
            self.joystick_direction_effect(msg.axes[0], msg.axes[1])

    def start_animation(self):
        self.is_running = False
        if self.animation_thread and self.animation_thread.is_alive():
            self.animation_thread.join()
        self.is_running = True
        self.animation_thread = threading.Thread(target=self.run_animation, daemon=True)
        self.animation_thread.start()

    def run_animation(self):
        while self.is_running:
            if self.pattern_index == 0:
                self.chase_effect()
            elif self.pattern_index == 1:
                self.windmill_effect()
            elif self.pattern_index == 2:
                self.pattern_effect()
            elif self.pattern_index == 3:
                self.constant_effect()

    def stop(self):
        self.is_running = False
        if self.animation_thread and self.animation_thread.is_alive():
            self.animation_thread.join()
        self.clear_leds()
        self.get_logger().info("LED Controller Shutting Down")

    def constant_effect(self):
        self.pixels.fill(COLORS.get(self.current_color, OFF))

    def chase_effect(self, delay=0.005):
        color_value = COLORS.get(self.current_color, OFF)
        width = 3

        for i in range(LED_COUNT):
            if not self.is_running:
                return
            
            for j in range(width):
                index = (i + j) % LED_COUNT
                self.pixels[index] = color_value
            
            time.sleep(delay)
            self.pixels[(i - 1) % LED_COUNT] = OFF

    def windmill_effect(self, delay=0.05, sections=4):
        color_value = COLORS.get(self.current_color, OFF)

        for pixel in range(LED_COUNT):
            if not self.is_running:
                return
            time.sleep(delay)
            index = pixel
            for section in range(sections):
                section_color = color_value if section % 2 == 0 else OFF
                self.pixels[(index + section * (LED_COUNT // sections)) % LED_COUNT] = section_color

    def pattern_effect(self, delay=0.2):
        color_value = COLORS.get(self.current_color, OFF)
        pixel_group = [i for i in range(0, LED_COUNT, 2)]

        for _ in range(4):
            if not self.is_running:
                return
            time.sleep(delay)
            self.pixels.fill(color_value)
            for pixel in pixel_group:
                self.pixels[pixel] = OFF
            time.sleep(delay)
            self.pixels.fill(color_value)
            for pixel in range(LED_COUNT):
                if pixel not in pixel_group:
                    self.pixels[pixel] = OFF

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
