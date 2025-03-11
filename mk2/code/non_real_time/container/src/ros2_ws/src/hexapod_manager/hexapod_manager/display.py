import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class HexapodTouchscreen(Node):
    def __init__(self):
        super().__init__('hexapod_touchscreen')
        self.publisher = self.create_publisher(Joy, '/joy', 10)
        
        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)  # Fullscreen mode
        pygame.display.set_caption("Hexapod Control")

        # Define button properties
        self.button_color = (0, 128, 255)
        self.font = pygame.font.Font(None, 72)  # Larger font size for title
        self.button_font = pygame.font.Font(None, 48)  # Larger font size for buttons
        self.scroll_font = pygame.font.Font(None, 36)  # Smaller font size for scrolling text
        
        # Define button positions and sizes
        self.mode_button_rect = pygame.Rect(100, 200, 240, 100)  # Larger "mode" button
        self.color_button_rect = pygame.Rect(400, 200, 240, 100)  # Larger "color" button
        
        self.scroll_text = "Manufactured Motion"
        self.scroll_pos = self.screen.get_width()  # Start position for scrolling text
        self.running = True

    def draw_buttons(self):
        # Draw the "mode" button
        pygame.draw.rect(self.screen, self.button_color, self.mode_button_rect)
        mode_text = self.button_font.render("Mode", True, (255, 255, 255))
        self.screen.blit(mode_text, (self.mode_button_rect.centerx - mode_text.get_width() // 2,
                                     self.mode_button_rect.centery - mode_text.get_height() // 2))  # Center text

        # Draw the "color" button
        pygame.draw.rect(self.screen, self.button_color, self.color_button_rect)
        color_text = self.button_font.render("Color", True, (255, 255, 255))
        self.screen.blit(color_text, (self.color_button_rect.centerx - color_text.get_width() // 2,
                                      self.color_button_rect.centery - color_text.get_height() // 2))  # Center text

        # Draw the title
        title_text = self.font.render("Hexapod Mark II", True, (255, 255, 255))
        self.screen.blit(title_text, (self.screen.get_width() // 2 - title_text.get_width() // 2, 50))  # Center title

        # Draw scrolling text at the bottom
        scroll_text_surface = self.scroll_font.render(self.scroll_text, True, (255, 255, 255))
        self.screen.blit(scroll_text_surface, (self.scroll_pos, self.screen.get_height() - 50))  # Position at the bottom

    def send_joy_message(self, button_index):
        msg = Joy()
        msg.buttons = [0] * 12  # Initialize all buttons to 0
        msg.buttons[button_index] = 1  # Press the specified button
        self.publisher.publish(msg)
        self.get_logger().info(f"Published to /joy topic with button {button_index} pressed")

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if self.mode_button_rect.collidepoint(event.pos):
                        self.send_joy_message(10)  # Send joy 10 for "mode"
                    elif self.color_button_rect.collidepoint(event.pos):
                        self.send_joy_message(11)  # Send joy 11 for "color"

            # Update scrolling text position
            self.scroll_pos -= 2  # Scroll speed
            if self.scroll_pos < -self.scroll_font.size(self.scroll_text)[0]:
                self.scroll_pos = self.screen.get_width()  # Reset position to start

            # Refresh screen
            self.screen.fill((0, 0, 0))  # Black background
            self.draw_buttons()
            pygame.display.flip()

        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = HexapodTouchscreen()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
