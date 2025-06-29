import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import board
import neopixel_spi as neopixel
import threading
import math

LED_COUNT = 60
PATTERN_COUNT = 5
LED_PIN = board.SPI()
DEBOUNCE_DELAY = 0.5
DEAD_ZONE = 0.1

OFF = (0, 0, 0)
COLORS = {
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
    "white": (255, 255, 255),
    "purple": (106, 27, 154),
    "yellow": (255, 255, 0),
}
COLOR_KEYS = list(COLORS.keys())


class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller')

        self.pixels = neopixel.NeoPixel_SPI(LED_PIN, LED_COUNT, pixel_order=neopixel.GRB, auto_write=False)

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.pattern_index = 0
        self.color_index = 0
        self.current_color = COLOR_KEYS[self.color_index]

        self.last_button_press_time = 0
        self.is_running = False
        self.animation_thread = None

        self.chase_pos = 0
        self.windmill_pos = 0
        self.pattern_state = 0
        self.last_x = 0.0
        self.last_y = 0.0

        self.prev_chase_indices = []
        self.prev_windmill_indices = []
        self.prev_pattern_indices_off = []

        self.pattern_effect_last_toggle = time.time()
        self.pattern_effect_toggle_interval = 0.5

        self.start_animation()

    def clear_leds(self):
        self.pixels.fill(OFF)
        self.pixels.show()

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
        self.pixels.show()

    def joy_callback(self, msg):
        current_time = time.time()

        if len(msg.buttons) > 10 and msg.buttons[10] == 1 and current_time - self.last_button_press_time >= DEBOUNCE_DELAY:
            self.last_button_press_time = current_time
            self.pattern_index = (self.pattern_index + 1) % PATTERN_COUNT
            self.reset_pattern_states()
            self.start_animation()

        if len(msg.buttons) > 11 and msg.buttons[11] == 1 and current_time - self.last_button_press_time >= DEBOUNCE_DELAY:
            self.last_button_press_time = current_time
            self.color_index = (self.color_index + 1) % len(COLOR_KEYS)
            self.current_color = COLOR_KEYS[self.color_index]
            self.start_animation()

        if self.pattern_index == 4 and len(msg.axes) >= 2:
            self.joystick_direction_effect(msg.axes[0], msg.axes[1])

    def reset_pattern_states(self):
        self.chase_pos = 0
        self.windmill_pos = 0
        self.pattern_state = 0

        self.prev_chase_indices = []
        self.prev_windmill_indices = []
        self.prev_pattern_indices_off = []

        self.pattern_effect_last_toggle = time.time()

    def start_animation(self):
        self.is_running = False
        if self.animation_thread and self.animation_thread.is_alive():
            self.animation_thread.join()

        if self.pattern_index < (PATTERN_COUNT - 1):
            self.is_running = True
            self.animation_thread = threading.Thread(target=self.run_animation, daemon=True)
            self.animation_thread.start()
        else:
            self.clear_leds()

    def run_animation(self):
        while self.is_running:
            if self.pattern_index == 0:
                self.chase_effect_step()
            elif self.pattern_index == 1:
                self.windmill_effect_step()
            elif self.pattern_index == 2:
                self.pattern_effect_step()
            elif self.pattern_index == 3:
                self.constant_effect_step()
            else:
                self.clear_leds()

            time.sleep(0.02)

    def chase_effect_step(self):
        color = COLORS.get(self.current_color, OFF)
        width = 3

        for idx in self.prev_chase_indices:
            self.pixels[idx] = OFF

        current_indices = [(self.chase_pos + i) % LED_COUNT for i in range(width)]

        for idx in current_indices:
            self.pixels[idx] = color

        self.prev_chase_indices = current_indices
        self.chase_pos = (self.chase_pos + 1) % LED_COUNT
        self.pixels.show()

    def windmill_effect_step(self):
        color = COLORS.get(self.current_color, OFF)
        sections = 4
        
        for idx in self.prev_windmill_indices:
            self.pixels[idx] = OFF

        current_indices = []
        for section in range(sections):
            index = (self.windmill_pos + section * (LED_COUNT // sections)) % LED_COUNT
            if section % 2 == 0:
                self.pixels[index] = color
                current_indices.append(index)
            else:
                self.pixels[index] = OFF

        self.prev_windmill_indices = current_indices
        self.windmill_pos = (self.windmill_pos + 1) % LED_COUNT
        self.pixels.show()

    def pattern_effect_step(self):
        now = time.time()
        if now - self.pattern_effect_last_toggle < self.pattern_effect_toggle_interval:
            color = COLORS.get(self.current_color, OFF)
            self.pixels.fill(color)
            for idx in self.prev_pattern_indices_off:
                self.pixels[idx] = OFF
            self.pixels.show()
            return

        self.pattern_effect_last_toggle = now

        color = COLORS.get(self.current_color, OFF)
        self.pixels.fill(color)

        for idx in self.prev_pattern_indices_off:
            self.pixels[idx] = color

        if self.pattern_state == 0:
            off_indices = list(range(0, LED_COUNT, 2))
        else:
            off_indices = list(range(1, LED_COUNT, 2))

        for idx in off_indices:
            self.pixels[idx] = OFF

        self.prev_pattern_indices_off = off_indices
        self.pattern_state = 1 - self.pattern_state
        self.pixels.show()

    def constant_effect_step(self):
        color = COLORS.get(self.current_color, OFF)
        self.pixels.fill(color)
        self.pixels.show()

    def stop(self):
        self.is_running = False
        if self.animation_thread and self.animation_thread.is_alive():
            self.animation_thread.join()
        self.clear_leds()

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
