import os
import pygame
import time

os.environ["SDL_VIDEODRIVER"] = "dummy"

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
    "BACK": "macro_2",
    "START": "macro_1",
    "HOME": None,
    "J1": "G0 X0 Y0 Z200 R0 P0 W0",
    "J2": None
}

def controller_check_loop(controller_queue):
    pygame.init()
    pygame.joystick.init()
    
    controller_connected = False

    while True:
        num_joysticks = pygame.joystick.get_count()
        if num_joysticks == 0:
            if controller_connected:
                controller_connected = False
                controller_queue.put("Err: No Controller Connected")
            time.sleep(1)  # Wait for a second before trying again
        else:
            if not controller_connected:
                controller_connected = True
                joystick_info = f"M118 Joystick 0: Name: {pygame.joystick.Joystick(0).get_name()} Axes: {pygame.joystick.Joystick(0).get_numaxes()} Buttons: {pygame.joystick.Joystick(0).get_numbuttons()}"
                controller_queue.put("Controller Connected")
                print(joystick_info)
            time.sleep(1)

if __name__ == "__main__":
    from multiprocessing import Queue
    controller_queue = Queue()
    controller_check_loop(controller_queue)