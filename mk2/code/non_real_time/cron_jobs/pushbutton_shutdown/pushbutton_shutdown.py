from gpiozero import Button
import subprocess
import sys
import time
import os

# Initialize the button with GPIO pin 17 and enable the internal pull-up resistor
button = Button(17, pull_up=True)

# Define the actions to take when the button is pressed or released
def on_button_press():
    print("Button pressed")

    try:
        subprocess.run([
            "docker", "exec", "-i", "hexapod_latest",
            "bash", "-c",
            "source /app/src/ros2_ws/install/setup.bash && "
            "python3 /app/src/ros2_ws/src/hexapod_manager/hexapod_manager/run_from_file.py "
            "/app/src/ros2_ws/resources/sit.txt"
        ])
    except Exception as e:
        print(f"Exception with sit script! {e}")

    subprocess.call(["shutdown", "now"])

# Attach the functions to the button press and release events
button.when_pressed = on_button_press

# Keep the program running to listen for button presses
while(True):
    time.sleep(1)
