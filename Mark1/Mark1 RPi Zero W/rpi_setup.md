# Raspberry Pi Zero W Configuration

This guide will walk you through how to setup your Raspberry Pi Zero W to have the necessary permissions, libraries, and code to be able to function with the Hexapod.

## Flashing the SD Card

1. [Get the Raspberry Pi Imaging Tool](https://www.raspberrypi.com/software/).
2. Choose `Raspberry Pi Zero W` for the `Raspberry Pi Device`.
3. Choose `Raspberry Pi OS Lite` for the `Operating System`.
4. Choose the appropriate MicroSD card for `Storage`.
5. Click `Next`.
6. Select `Edit Settings` for `Would you like to apply OS customisation settings`.
7. Set `hostname` to be `hexapod<lastname>.local` -- this will prevent issue if multiple Hexapods are in the same area/network.
8. Set your `username` and `password` as appropriate.
9. Configure `Wireless LAN` and `Set Locale Settings` as necessary.
10. Navigate to `Services`.
11. Check `Enable SSH` and `Use password authentication`.
12. `Save` and flash the SD card!

## Configuring the Raspberry Pi

1. SSH into the Raspberry Pi using the information above. `SSH <username>@hexapod<lastname>.local`.
2. Enter `sudo raspi-config` in terminal.
3. Select `Interface Options` then select `I6 Serial Port`.
4. `Would you like a login shell to be accessible over serial?` --> `No`.
5. `Would you like the serial port hardware to be enabled?` --> `Yes`.
6. Exit out.
7. Run `sudo reboot` and log back in.

## Serial and WebServer

1. Run `sudo apt-get update`
2. Run `sudo apt-get install python3-serial`
3. Run `sudo apt-get install flask` or `sudo apt-get install pip` --> `pip install flask`
4. Run `sudo apt-get upgrade`

### Testing Serial

- [testing_serial.py](test_files/testing_serial.py) & [testing_serial.ino](test_files/testing_serial.ino)
  - Used to ensure your Raspberry Pi is configured properly and that the Raspberry pi can take command via SSH and pass them via serial to the Teensy 4.1.
  - Usage: `python3 raspi_serial.py "Hello World"`
  - If you aren't able to run the file, make sure it is an executable by doing: `chmod +x raspi_serial.py`

## Controller Setup

0. Run `sudo apt update`
1. Run `sudo apt install git`
2. Run `sudo apt install cmake`
3. Run `sudo apt install dkms`
4. Run `sudo apt install libevdev-dev`
5. Run `sudo apt install libudev-dev`
6. Run `sudo apt install libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev`
7. Run `pip install --upgrade --force-reinstall pygame`

Follow the instructions listed [here](https://retropie.org.uk/docs/Nintendo-Switch-Controllers/)

### Pairing

1. Run `sudo bluetoothctl`
2. Enter `agent on;`
3. Enter `default-agent;`
4. Enter `scan on;`
5. On the Forty4 controller press `Y` + `home` to enter pairing mode
6. Look for the value similar to: `[NEW] Device 0C:FC:83:19:C2:A3 Pro Controller`
7. Enter `pair 0C:FC:83:19:C2:A3;`
8. If successful, enter `trust 0C:FC:83:19:C2:A3;`
9. Enter `exit`
10. Run `ls /dev/input/js*`. If you see something similar to `/dev/input/js0` it is successful. If it is not, return to `sudo bluetoothctl` to authorize or debug.

### Testing Bluetooth Controller

Once completed, run [controller_test.py](test_files/controller_test.py) to make sure multiple joysticks and axes are detected. Before running the program, make sure your controller says `SW1` -- If it says `SW4` mash multiple buttons on the controller until it switches to `SW4`. Running the program should return:

```terminal
Joystick 0:
  Name: Nintendo Switch Pro Controller
  Axes: 4
  Buttons: 14
Joystick 1:
  Name: Nintendo Switch Pro Controller IMU
  Axes: 6
  Buttons: 0
```
