import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame

def init_controller():
    # Initialize Pygame
    pygame.init()
    
    # Initialize the joystick module
    pygame.joystick.init()

    # Check if any joysticks/controllers are connected
    num_joysticks = pygame.joystick.get_count()
    print("Number of connected joysticks:", num_joysticks)
    if num_joysticks == 0:
        print("No controllers connected.")
        return None

    # Initialize the first joystick
    controller = pygame.joystick.Joystick(0)
    controller.init()
    print("Controller initialized:", controller.get_name())
    
    return controller

def main():
    controller = init_controller()
    if not controller:
        return

    try:
        while True:
            # Check events
            pygame.event.pump()
            
            # Read axes data
            num_axes = controller.get_numaxes()
            axes_data = [controller.get_axis(i) for i in range(num_axes)]
            print("Axes data:", axes_data)
            
            # Read button data
            num_buttons = controller.get_numbuttons()
            buttons_data = [controller.get_button(i) for i in range(num_buttons)]
            print("Button data:", buttons_data)

    except KeyboardInterrupt:
        # Clean up
        pygame.quit()

if __name__ == "__main__":
    main()
