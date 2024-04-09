import time
import serial
from controller_setup import *
from webserver_setup import start_server, command_queue as web_command_queue, serial_data_queue
from multiprocessing import Process, Queue

ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
poll_rate = 10  # Polling 10 times per second
polling_interval = 1.0 / poll_rate
teensy_command_queue = Queue()

def send_to_teensy(data):
    ser.write((data + '\n').encode())

def read_from_serial(serial_queue, serial_data_queue):
    while True:
        if ser.in_waiting > 0:
            serial_data = ser.readline().decode().strip()
            serial_queue.put(serial_data)
            serial_data_queue.put(serial_data)  # Put data into the serial_data_queue as well
            print(serial_data)

def main():
    controller, controller_status =  init_controller()
    send_to_teensy(controller_status)

    serial_queue = Queue()
    serial_process = Process(target=read_from_serial, args=(serial_queue, serial_data_queue))
    serial_process.daemon = True
    serial_process.start()

    try:
        while True:
            start_time = time.time()
            pygame.event.pump()

            if not web_command_queue.empty():
                send_to_teensy(web_command_queue.get())

            axes_data = [round(controller.get_axis(i), 2) for i in range(controller.get_numaxes())]
            axes_data[1] = -axes_data[1]
            axes_data[3] = -axes_data[3]
            joystick_out = " G1 X" + str(axes_data[0]) + " Y" + str(axes_data[1])

            num_buttons = controller.get_numbuttons()
            buttons_data = [controller.get_button(i) for i in range(num_buttons)]

            # Button mapping is defined in controller_setup.py
            button_out = ""
            for button, action in button_mapping.items():
                button_index = list(button_mapping.keys()).index(button)
                if action is not None and buttons_data[button_index] != 0:
                    button_out += f" {action}"

            try:
                if (axes_data[0] or axes_data[1]):
                    send_to_teensy(joystick_out)
                if any(x != 0 for x in buttons_data) and button_out:
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
    start_server()
    main()
