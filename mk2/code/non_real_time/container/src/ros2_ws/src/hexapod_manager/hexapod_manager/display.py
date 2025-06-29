import pygame
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class HexapodTouchscreen(Node):
    def __init__(self):
        super().__init__('hexapod_touchscreen')
        self.publisher = self.create_publisher(Joy, '/joy', 10)
        
        pygame.init()
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        pygame.display.set_caption("Hexapod Control")

        self.button_color = (0, 128, 255)
        self.font = pygame.font.Font(None, 72)
        self.button_font = pygame.font.Font(None, 48)
        self.scroll_font = pygame.font.Font(None, 36)
        
        screen_width, screen_height = self.screen.get_size()
        button_width, button_height = 240, 100
        
        self.mode_button_rect = pygame.Rect((screen_width // 2 - button_width - 20, screen_height // 2 - button_height // 2), (button_width, button_height))
        self.color_button_rect = pygame.Rect((screen_width // 2 + 20, screen_height // 2 - button_height // 2), (button_width, button_height))
        
        self.status_text = "Waiting for voltage update..."
        self.battery_txt_color = (255, 255, 255)
        self.create_subscription(String, 'hexapod_status', self.status_callback, 1)
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

        self.scroll_text = "Manufactured Motion"
        self.scroll_pos = self.screen.get_width()
        self.running = True

    def draw_buttons(self):
        pygame.draw.rect(self.screen, self.button_color, self.mode_button_rect)
        mode_text = self.button_font.render("Mode", True, (255, 255, 255))
        self.screen.blit(mode_text, (self.mode_button_rect.centerx - mode_text.get_width() // 2,
                                     self.mode_button_rect.centery - mode_text.get_height() // 2))

        pygame.draw.rect(self.screen, self.button_color, self.color_button_rect)
        color_text = self.button_font.render("Color", True, (255, 255, 255))
        self.screen.blit(color_text, (self.color_button_rect.centerx - color_text.get_width() // 2,
                                      self.color_button_rect.centery - color_text.get_height() // 2))

        title_text = self.font.render("Hexapod Mark II", True, (255, 255, 255))
        self.screen.blit(title_text, (self.screen.get_width() // 2 - title_text.get_width() // 2, 50))

        scroll_text_surface = self.scroll_font.render(self.scroll_text, True, (255, 255, 255))
        self.screen.blit(scroll_text_surface, (self.scroll_pos, self.screen.get_height() - 50))

        battery_txt_surface = self.font.render(self.status_text, True, self.battery_txt_color)
        self.screen.blit(battery_txt_surface, (self.screen.get_width() // 2 - battery_txt_surface.get_width() // 2, self.screen.get_height() // 2 + 150))

    def send_joy_message(self, button_index):
        msg = Joy()
        msg.buttons = [0] * 12
        msg.buttons[button_index] = 1
        self.publisher.publish(msg)

    def run(self):
        clock = pygame.time.Clock()
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if self.mode_button_rect.collidepoint(event.pos):
                        self.send_joy_message(10)
                    elif self.color_button_rect.collidepoint(event.pos):
                        self.send_joy_message(11)

            self.scroll_pos -= 2
            if self.scroll_pos < -self.scroll_font.size(self.scroll_text)[0]:
                self.scroll_pos = self.screen.get_width()

            self.screen.fill((0, 0, 0))
            self.draw_buttons()
            pygame.display.flip()

        pygame.quit()

    def status_callback(self, msg):
        self.status_text = msg.data
        percent_str = self.status_text.split(":")[1].strip().replace('%', '')
        voltage_percent = int(percent_str)
        self.get_logger().info(f"got msg, {self.status_text}")
        if voltage_percent > 20:
            self.battery_txt_color = (0, 255, 0)
        else:
            self.battery_txt_color = (255, 0, 0)
        

def main(args=None):
    rclpy.init(args=args)
    node = HexapodTouchscreen()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
