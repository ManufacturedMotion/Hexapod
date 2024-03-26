import pygame

def main():
    # Initialize Pygame
    pygame.init()

    # Initialize the joystick module
    pygame.joystick.init()

    # Check if any joysticks/controllers are connected
    num_joysticks = pygame.joystick.get_count()
    print("Number of connected joysticks:", num_joysticks)

    # Print information about each connected joystick
    for i in range(num_joysticks):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        print("Joystick {}:".format(i))
        print("  Name:", joystick.get_name())
        print("  Axes:", joystick.get_numaxes())
        print("  Buttons:", joystick.get_numbuttons())

    # Quit Pygame
    pygame.quit()

if __name__ == "__main__":
    main()