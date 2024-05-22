import time
import serial
from controller_setup import controller_check_loop, button_mapping
from webserver_setup import start_server, command_queue as web_command_queue, serial_data_queue
from multiprocessing import Process, Queue
import pygame

ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
poll_rate = 20  # Polling 10 times per second
polling_interval = 1.0 / poll_rate
teensy_command_queue = Queue()

def send_to_teensy(data):
    try:
        ser.write((data + '\n').encode())
        ser.flush()
        print(f"Data: \"{data}\" sent successfully")
    except Exception as e:
        print(f"Failed to send data: {e}")

def read_from_serial(serial_queue, serial_data_queue):
    while True:
        if ser.in_waiting > 0:
            serial_data = ser.readline().decode().strip()
            serial_queue.put(serial_data)
            serial_data_queue.put(serial_data)
            print(f"Received from Teensy: {serial_data}")

def main():
    send_to_teensy("program running")
    serial_queue = Queue()
    serial_process = Process(target=read_from_serial, args=(serial_queue, serial_data_queue))
    serial_process.daemon = True
    serial_process.start()

    controller_queue = Queue()
    controller_process = Process(target=controller_check_loop, args=(controller_queue,))
    controller_process.daemon = True
    controller_process.start()

    pygame.init()  # Initialize pygame in the main process
    pygame.joystick.init()

    controller = None

    try:
        while True:
            start_time = time.time()
            
            # Check for messages from the controller process
            if not controller_queue.empty():
                controller_status = controller_queue.get()
                send_to_teensy(controller_status)
                if controller_status == "Controller Connected":
                    if pygame.joystick.get_count() > 0:
                        controller = pygame.joystick.Joystick(0)
                        controller.init()
                        print("Controller connected and initialized.")
                elif "Err" in controller_status:
                    print(controller_status)
                    controller = None

            # Check for new controllers if none are connected
            if controller is None:
                pygame.joystick.quit()
                pygame.joystick.init()
                if pygame.joystick.get_count() > 0:
                    controller = pygame.joystick.Joystick(0)
                    controller.init()
                    send_to_teensy("Controller Connected")
                    print("Controller connected and initialized.")

            if controller is not None:
                try:
                    pygame.event.pump()
                    joystick_out = ""
                    button_out = ""
                    macro_file = ""
                    execute_macro = False

                    if not web_command_queue.empty():
                        send_to_teensy(web_command_queue.get())

                    try:
                        axes_data = [round(controller.get_axis(i), 2) for i in range(controller.get_numaxes())]
                        axes_data[1] = -axes_data[1]
                        axes_data[3] = -axes_data[3]
                        joystick_out = "G1 X" + str(axes_data[0]) + " Y" + str(axes_data[1])

                        num_buttons = controller.get_numbuttons()
                        buttons_data = [controller.get_button(i) for i in range(num_buttons)]

                        for button, action in button_mapping.items():
                            button_index = list(button_mapping.keys()).index(button)
                            if action is not None and buttons_data[button_index] != 0:
                                button_out += f'{action}'
                                if "macro" in button_out:
                                    macro_file = button_out + '.txt'
                                    execute_macro = True

                        if execute_macro:
                            try:
                                with open(macro_file, 'r') as file:
                                    for line in file:
                                        send_to_teensy(line.strip())
                                        time.sleep(0.05)
                                execute_macro = False
                                button_out = ""
                            except FileNotFoundError:
                                print("Macro file not found!")

                        if (axes_data[0] or axes_data[1]):
                            send_to_teensy(joystick_out)
                        if any(x != 0 for x in buttons_data) and button_out:
                            send_to_teensy(button_out)
                    except Exception as e:
                        print(f"Error: {e}")
                except pygame.error:
                    print("Controller disconnected.")
                    controller = None  # Reset controller to None if disconnected

            elapsed_time = time.time() - start_time
            if elapsed_time < polling_interval:
                time.sleep(polling_interval - elapsed_time)

    except KeyboardInterrupt:
        ser.close()
        pygame.quit()

if __name__ == "__main__":
    start_server()
    main()
