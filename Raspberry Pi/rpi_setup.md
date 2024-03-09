# Raspberry Pi Zero W Configuration

This guide will walk you through how to setup your Raspberry Pi Zero W to have the necessary permissions, libraries, and code to be able to function with the Hexapod.

## Flashing the SD Card

1. [Get the Raspberry Pi Imaging Tool](https://www.raspberrypi.com/software/).
2. Choose `Raspberry Pi Zero 2 W` for the `Raspberry Pi Device`.
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

## Installing Libraries

1. Run `sudo apt-get update`
2. Run `sudo apt-get install python3-serial`

## Example Files

### Testing Serial

- [testing_serial.py](Raspberry\%Pi/rpi_setup.md) & [testing_serial.ino](Raspberry\%Pi/testing_serial.ino)
  - Used to ensure your Raspberry Pi is configured properly and that the Raspberry pi can take command via SSH and pass them via serial to the Teensy 4.1.
  - Usage: `python3 raspi_serial.py "Hello World"`
  - If you aren't able to run the file, make sure it is an executable by doing: `chmod +x raspi_serial.py`
