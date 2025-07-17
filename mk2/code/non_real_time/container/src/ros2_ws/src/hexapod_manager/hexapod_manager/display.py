import pygame
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory

class HexapodTouchscreen(Node):
    # === INITIALIZATION ===
    def __init__(self):
        super().__init__('hexapod_touchscreen')
        self.publisher = self.create_publisher(Joy, '/joy', 10)
        
        pygame.init()
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        pygame.display.set_caption("Hexapod Control")

        try:
            package_share_directory = get_package_share_directory('hexapod_manager')
            font_path = os.path.join(package_share_directory, 'resource', 'mokoto.regular.ttf')
            print(f"Looking for font at: {font_path}")
            
            if os.path.exists(font_path):
                self.font = pygame.font.Font(font_path, 72)
                self.title_font = pygame.font.Font(font_path, 96)
            else:
                raise FileNotFoundError("Font file not found")
        except (FileNotFoundError, Exception) as e:
            print(f"Could not load Mokoto font: {e}")
            self.font = pygame.font.Font(None, 72)
            self.title_font = pygame.font.Font(None, 96)
        
        screen_width, screen_height = self.screen.get_size()
        
        battery_margin = 40
        battery_width = 100
        battery_height = 50
        self.battery_x = screen_width - battery_width - battery_margin
        self.battery_y = battery_margin
        self.battery_width = battery_width
        self.battery_height = battery_height
        
        title_font_temp = pygame.font.Font(None, 96)
        title_text_temp = title_font_temp.render("HexII", True, (255, 255, 255))
        
        bottom_margin = 50
        title_y = screen_height - bottom_margin - title_text_temp.get_height()
        
        self.title_y = title_y
        
        self.status_text = "Waiting for voltage update..."
        self.battery_txt_color = (255, 255, 255)
        self.battery_percentage = None
        self.create_subscription(String, 'hexapod_status', self.status_callback, 1)
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

        self.running = True
        
        self.load_svg_arrow()

    # === UTILITY FUNCTIONS ===
    def get_font_path(self):
        """Get the path to the Mokoto font file"""
        try:
            package_share_directory = get_package_share_directory('hexapod_manager')
            font_path = os.path.join(package_share_directory, 'resource', 'mokoto.regular.ttf')
            if os.path.exists(font_path):
                return font_path
        except Exception:
            pass
        return None

    # === BATTERY DISPLAY ===
    def draw_battery_icon(self):
        """Draw a horizontal battery icon in the top right corner with voltage percentage or '?' if waiting"""
        screen_width, screen_height = self.screen.get_size()
        
        battery_x = self.battery_x
        battery_y = self.battery_y
        battery_width = self.battery_width
        battery_height = self.battery_height
        
        terminal_width = 12
        terminal_height = 30
        terminal_x = battery_x + battery_width
        terminal_y = battery_y + (battery_height - terminal_height) // 2
        
        battery_rect = pygame.Rect(battery_x, battery_y, battery_width, battery_height)
        terminal_rect = pygame.Rect(terminal_x, terminal_y, terminal_width, terminal_height)
        
        pygame.draw.rect(self.screen, (255, 255, 255), battery_rect, 3)
        pygame.draw.rect(self.screen, (255, 255, 255), terminal_rect)
        
        if self.battery_percentage is not None:
            fill_width = int((battery_width - 6) * (self.battery_percentage / 100))
            fill_rect = pygame.Rect(battery_x + 3, battery_y + 3, fill_width, battery_height - 6)
            
            if self.battery_percentage >= 40:
                fill_color = (0, 255, 0)
            elif self.battery_percentage >= 30:
                fill_color = (255, 255, 0)
            elif self.battery_percentage >= 20:
                fill_color = (255, 128, 0)
            else:
                fill_color = (255, 0, 0)
            
            print(f"Battery: {self.battery_percentage}% - Color: {fill_color}")
                
            pygame.draw.rect(self.screen, fill_color, fill_rect)
            
            font_path = self.get_font_path()
            if font_path:
                battery_font = pygame.font.Font(font_path, 16)
            else:
                battery_font = pygame.font.Font(None, 16)
            text = f"{self.battery_percentage}%"
            
            text_surface = battery_font.render(text, True, (255, 255, 255))
            text_x = battery_x + (battery_width - text_surface.get_width()) // 2
            text_y = battery_y + (battery_height - text_surface.get_height()) // 2
            
            black_outline = battery_font.render(text, True, (0, 0, 0))
            for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1), (-1, 0), (1, 0), (0, -1), (0, 1)]:
                self.screen.blit(black_outline, (text_x + dx, text_y + dy))
            
            self.screen.blit(text_surface, (text_x, text_y))
        else:
            font_path = self.get_font_path()
            if font_path:
                question_font = pygame.font.Font(font_path, 24)
            else:
                question_font = pygame.font.Font(None, 24)
            
            question_surface = question_font.render("?", True, (255, 255, 255))
            question_x = battery_x + (battery_width - question_surface.get_width()) // 2
            question_y = battery_y + (battery_height - question_surface.get_height()) // 2
            
            question_black = question_font.render("?", True, (0, 0, 0))
            for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1), (-1, 0), (1, 0), (0, -1), (0, 1)]:
                self.screen.blit(question_black, (question_x + dx, question_y + dy))
            
            self.screen.blit(question_surface, (question_x, question_y))

    # === ARROW DISPLAY ===
    def load_svg_arrow(self):
        """Load PNG arrow and convert to pygame surface"""
        try:
            package_share_directory = get_package_share_directory('hexapod_manager')
            png_path = os.path.join(package_share_directory, 'resource', 'manufactured_motion_arrow.png')
            
            print(f"Looking for PNG at: {png_path}")
            
            if os.path.exists(png_path):
                self.arrow_surface = pygame.image.load(png_path).convert_alpha()
                desired_size = (240, 320)
                self.arrow_surface = pygame.transform.scale(self.arrow_surface, desired_size)
                print(f"Successfully loaded PNG: {png_path}")
            else:
                self.arrow_surface = None
                print(f"PNG file not found: {png_path}")
                print(f"Package share directory: {package_share_directory}")
                try:
                    resource_dir = os.path.join(package_share_directory, 'resource')
                    if os.path.exists(resource_dir):
                        contents = os.listdir(resource_dir)
                        print(f"Contents of resource directory: {contents}")
                    else:
                        print("Resource directory does not exist")
                except Exception as list_error:
                    print(f"Could not list resource directory: {list_error}")
        except Exception as e:
            print(f"Error loading PNG: {e}")
            self.arrow_surface = None

    def draw_arrow_up(self):
        """Draw SVG arrow or fallback to drawn arrow in the upper center of the screen"""
        screen_width, screen_height = self.screen.get_size()
        center_x = screen_width // 2
        
        if self.arrow_surface:
            arrow_rect = self.arrow_surface.get_rect()
            arrow_rect.centerx = center_x
            arrow_rect.top = 40
            self.screen.blit(self.arrow_surface, arrow_rect)
        else:
            arrow_width = 240
            arrow_height = 320
            shaft_width = 90
            
            arrow_top = 40
            arrow_center_y = arrow_top + arrow_height // 2
            
            head_points = [
                (center_x, arrow_top),
                (center_x - arrow_width // 2, arrow_center_y - arrow_height // 4),
                (center_x + arrow_width // 2, arrow_center_y - arrow_height // 4)
            ]
            
            shaft_rect = pygame.Rect(
                center_x - shaft_width // 2,
                arrow_center_y - arrow_height // 4,
                shaft_width,
                arrow_height // 2
            )
            
            pygame.draw.polygon(self.screen, (255, 255, 255), head_points)
            pygame.draw.rect(self.screen, (255, 255, 255), shaft_rect)
            
            pygame.draw.polygon(self.screen, (200, 200, 200), head_points, 3)
            pygame.draw.rect(self.screen, (200, 200, 200), shaft_rect, 3)

    # === MAIN DRAWING ===
    def draw_buttons(self):
        self.draw_battery_icon()
        self.draw_arrow_up()

        title_text = self.title_font.render("HexII", True, (255, 255, 255))
        title_x = (self.screen.get_width() - title_text.get_width()) // 2
        self.screen.blit(title_text, (title_x, self.title_y))

    # === MESSAGE HANDLING ===
    def send_joy_message(self, button_index):
        msg = Joy()
        msg.buttons = [0] * 12
        msg.buttons[button_index] = 1
        self.publisher.publish(msg)

    # === MAIN LOOP ===
    def run(self):
        clock = pygame.time.Clock()
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.screen.fill((0, 0, 0))
            self.draw_buttons()
            pygame.display.flip()

        pygame.quit()

    # === ROS CALLBACKS ===
    def status_callback(self, msg):
        self.status_text = msg.data
        try:
            percent_str = self.status_text.split(":")[1].strip().replace('%', '')
            self.battery_percentage = int(percent_str)
            self.get_logger().info(f"got msg, {self.status_text}")
            if self.battery_percentage > 20:
                self.battery_txt_color = (0, 255, 0)
            else:
                self.battery_txt_color = (255, 0, 0)
        except (IndexError, ValueError):
            self.battery_percentage = None
            self.get_logger().warning(f"Could not parse battery percentage from: {self.status_text}")
        

def main(args=None):
    rclpy.init(args=args)
    node = HexapodTouchscreen()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
