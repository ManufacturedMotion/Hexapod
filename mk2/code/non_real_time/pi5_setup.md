Welcome to the Hexapod Command system for the Mk2 platform. This guide will walk you through configuring the Raspberry Pi 5 with Ubuntu, Docker, and Ros2. 

## Requirements
- Raspberry Pi 5
- Micro SD Card (64GB+ preferred).
- Micro HDMI to HDMI cable.
- USB connected keyboard.
- USB connected mouse.

## 1. Flashing the SD Card

1. [Download the Raspberry Pi imager](https://www.raspberrypi.com/software/)
2. Select `Raspberry Pi 5` as the board
3. Select `Other general-purpose OS` -> `Ubuntu` -> `Ubuntu Desktop 24.04.1 LTS (64-BIT)`
4. Select your micro SD card.
5. Click save + wait.
6. Once flashed, plug your micro SD card into your Pi and connect peripherals.

## 2. Install Docker

1. Follow the Ubuntu setup instructions on first boot. We recommend naming your profile `hexapod-<lastname>` on the system `hexapod<lastname>`. If you run into issues setting up the pi, skip the Wi-Fi connection step until the system is fully configured.
2. Follow the instructions at [Install Docker Engine on Ubuntu - Install using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).
3. Once installed follow [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/) to run Docker commands without root privileges.
4. Run `apt install docker-compose`.

## 3. Download the Docker Image

1. Download the code in [Container Folder](./container). This code does not need to live anywhere specific, but having it in `Documents` is recommended.
2. Run `./build.sh`.
3. Run `./init_dev.sh` or `./init_prod.sh` depending on interfaces required.

## 4. Run setup script to install other dependencies and create custom cronjobs

1. Navigate to your mk2/non_realtime directory
2. run `sudo ./setup.sh`

## 5. Connect Xbox Core (Series S, X, etc) Controller

1. Controllers must have their firmware updated via the Xbox Accessories Windows app to be identified by recent Ubuntu releases.
2. Once updated, the controller can be connected via Ubuntu's bluetooth UI.
