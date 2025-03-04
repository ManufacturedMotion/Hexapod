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
        self.is_running = False
        self.last_button_press_time = 0
        self.color_index = 0
        self.current_color = COLOR_KEYS[self.color_index]
        
        self.animation_thread = None
        self.start_animation()

    def clear_leds(self):
        self.pixels.fill(OFF)

    def constant_effect(self):
        self.pixels.fill(COLORS.get(self.current_color, OFF))

    def chase_effect(self, delay=0.005, reverse=False):
        color_value = COLORS.get(self.current_color, OFF)
        width = 3
        
        for i in range(LED_COUNT):
            if not self.is_running:
                return
            
            for j in range(width):
                index = (-i - j) % LED_COUNT if reverse else (i + j) % LED_COUNT
                self.pixels[index] = color_value
            
            time.sleep(delay)
            
            tail_index = (-i - width) % LED_COUNT if reverse else (i - 1) % LED_COUNT
            self.pixels[tail_index] = OFF

    def windmill_effect(self, delay=0.05, sections=4, reverse=False):
        color_value = COLORS.get(self.current_color, OFF)
        
        for pixel in range(LED_COUNT):
            if not self.is_running:
                return
            time.sleep(delay)
            
            index = LED_COUNT - 1 - pixel if reverse else pixel
            
            for section in range(sections):
                section_color = color_value if section % 2 == 0 else OFF
                section_index = (index + section * (LED_COUNT // sections)) % LED_COUNT
                self.pixels[section_index] = section_color

    def pattern_effect(self, width=1, delay=0.2):
        color_value = COLORS.get(self.current_color, OFF)
        pixel_group = [i for i in range(0, LED_COUNT, 2 * width)]
        
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

    def joystick_direction_effect(self, x_axis, y_axis):
        # Clear LEDs before lighting up new ones
        self.clear_leds()

        # Calculate number of LEDs to light up in the direction of the joystick
        led_count = 5
        start_index = 0

        # For horizontal movement (X-axis)
        if x_axis > 0.5:  # Move right
            for i in range(led_count):
                self.pixels[(start_index + i) % LED_COUNT] = COLORS.get(self.current_color, OFF)
        elif x_axis < -0.5:  # Move left
            for i in range(led_count):
                self.pixels[(LED_COUNT - led_count + i) % LED_COUNT] = COLORS.get(self.current_color, OFF)

        # For vertical movement (Y-axis)
        elif y_axis > 0.5:  # Move up
            for i in range(led_count):
                self.pixels[i] = COLORS.get(self.current_color, OFF)
        elif y_axis < -0.5:  # Move down
            for i in range(led_count):
                self.pixels[LED_COUNT - 1 - i] = COLORS.get(self.current_color, OFF)

        # Ensure the LEDs light up in a circular fashion
        self.pixels.show()

    def joy_callback(self, msg):
        current_time = time.time()
        
        # Toggle pattern with buttons
        if len(msg.buttons) > 10 and msg.buttons[10] == 1:
            if current_time - self.last_button_press_time < DEBOUNCE_DELAY:
                return
            
            self.last_button_press_time = current_time
            self.pattern_index = (self.pattern_index + 1) % 5  # Add new mode 4
            self.start_animation()
        
        if len(msg.buttons) > 11 and msg.buttons[11] == 1:
            if current_time - self.last_button_press_time < DEBOUNCE_DELAY:
                return
            
            self.last_button_press_time = current_time
            self.color_index = (self.color_index + 1) % len(COLOR_KEYS)
            self.current_color = COLOR_KEYS[self.color_index]
            self.start_animation()

        # Get axes values
        x_axis = msg.axes[0]  # Joystick X-axis
        y_axis = msg.axes[1]  # Joystick Y-axis
        
        # Call the joystick direction effect if the pattern index is 4
        if self.pattern_index == 4:
            self.joystick_direction_effect(x_axis, y_axis)

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
            elif self.pattern_index == 4:
                # Joystick direction mode active
                pass

    def stop(self):
        self.is_running = False
        if self.animation_thread and self.animation_thread.is_alive():
            self.animation_thread.join()
        self.clear_leds()
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