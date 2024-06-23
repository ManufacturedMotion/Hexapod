import pygame

def main():
    pygame.init()
    pygame.joystick.init()
    num_joysticks = pygame.joystick.get_count()
    print("Number of connected joysticks:", num_joysticks)
    
    for i in range(num_joysticks):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        print("Joystick {}:".format(i))
        print("  Name:", joystick.get_name())
        print("  Axes:", joystick.get_numaxes())
        print("  Buttons:", joystick.get_numbuttons())

    pygame.quit()

if __name__ == "__main__":
    main()