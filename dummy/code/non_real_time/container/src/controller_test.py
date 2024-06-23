import pygame
import time

def main():
    pygame.init()
    pygame.joystick.init()
    num_joysticks = pygame.joystick.get_count()
    print("Number of connected joysticks:", num_joysticks)
    
    joysticks = []
    
    for i in range(num_joysticks):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        joysticks.append(joystick)
        print("Joystick {}:".format(i))
        print("  Name:", joystick.get_name())
        print("  Axes:", joystick.get_numaxes())
        print("  Buttons:", joystick.get_numbuttons())

    running = True
    while running:
        pygame.event.pump()  # Process events
        for joystick in joysticks:
            # Get axis values
            axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            print("Joystick {}: Axes - {}".format(joysticks.index(joystick), axes))

            # Get button states
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            print("Joystick {}: Buttons - {}".format(joysticks.index(joystick), buttons))

        time.sleep(0.1)

        # Check for quit event
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    main()
