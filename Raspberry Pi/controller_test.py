import os
import pygame
import time
import serial

os.environ["SDL_VIDEODRIVER"] = "dummy"
ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)

def init_controller():
    pygame.init()
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

def send_to_teensy(data):
    # Append a newline character at the end of the data
    ser.write((data + '\n').encode())

def main():
    controller = init_controller()
    if not controller:
        return

    # Set the polling rate (in Hz)
    poll_rate = 10  # Polling 10 times per second
    polling_interval = 1.0 / poll_rate

    try:
        while True:
            start_time = time.time()
            pygame.event.pump()
            
            # Read axes data
            num_axes = controller.get_numaxes()
            axes_data = [round(controller.get_axis(i),2) for i in range(num_axes)]
            axes_data[1] = -axes_data[1]
            axes_data[3] = -axes_data[3]
            joystick_out = "G1 J1X" + str(axes_data[0]) + " J1Y" + str(axes_data[1]) + " J2X" + str(axes_data[2]) + " J2Y" + str(axes_data[3])
            print(joystick_out)
            
            num_buttons = controller.get_numbuttons()
            buttons_data = [controller.get_button(i) for i in range(num_buttons)]
            
            button_out = ("Button A" + str(buttons_data[0]) +
              " B" + str(buttons_data[1]) +
              " Y" + str(buttons_data[2]) +
              " X" + str(buttons_data[3]) +
              " LB" + str(buttons_data[5]) +
              " RB" + str(buttons_data[6]) +
              " LT" + str(buttons_data[7]) +
              " RT" + str(buttons_data[8]) +
              " BACK" +  str(buttons_data[9]) +
              " START" +  str(buttons_data[10]) +
              " HOME" +  str(buttons_data[11]) +
              " TURBO" +  str(buttons_data[4]) +
              " J1" +  str(buttons_data[12]) +
              " J2" +  str(buttons_data[13]))

            print(button_out)

            try:
                if any(x != 0 for x in axes_data):
                    send_to_teensy(joystick_out)
                if any(x != 0 for x in buttons_data):
                    send_to_teensy(button_out)
            except Exception as e:
                print(f"Error: {e}")
            
            # Wait for next polling interval
            elapsed_time = time.time() - start_time
            if elapsed_time < polling_interval:
                time.sleep(polling_interval - elapsed_time)

    except KeyboardInterrupt:
        ser.close()
        pygame.quit()

if __name__ == "__main__":
    main()

