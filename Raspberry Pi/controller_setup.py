import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame

button_mapping = {
    "A": "P1",
    "B": None,
    "Y": "P0",
    "X": "P2",
    "TURBO": None,
    "LB": None,
    "RB": None,
    "LT": None,
    "RT": None,
    "BACK": None,
    "START": None,
    "HOME": None,
    "J1": "G0 X0 Y0 Z200 R0 P0 W0",
    "J2": None
}

def init_controller():
    pygame.init()
    pygame.joystick.init()

    num_joysticks = pygame.joystick.get_count()
    joystick_info = "M118 Joysticks:" + str(num_joysticks)
    for i in range(num_joysticks):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        joystick_info += "\nM118 Joystick {}:".format(i)
        joystick_info += "  Name: {}".format(joystick.get_name())
        joystick_info += "  Axes: {}".format(joystick.get_numaxes())
        joystick_info += "  Buttons: {}".format(joystick.get_numbuttons())

    if num_joysticks == 0:
        joystick_info = "Err: No Controller Connected"

    # Initialize the first joystick
    controller = pygame.joystick.Joystick(0)
    controller.init()

    return controller, joystick_info
