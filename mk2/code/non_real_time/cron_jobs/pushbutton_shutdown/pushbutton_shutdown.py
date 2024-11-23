from gpiozero import Button
import subprocess
import sys
import time

# Initialize the button with GPIO pin 17 and enable the internal pull-up resistor
button = Button(17, pull_up=True)

# Define the actions to take when the button is pressed or released
def on_button_press():
    print("Button pressed")
    subprocess.call(["shutdown", "now"])

# Attach the functions to the button press and release events
button.when_pressed = on_button_press

# Keep the program running to listen for button presses
while(True):
    time.sleep(1)
