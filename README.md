# Hexapod

![Manufactured Motion Logo](Images/banner_w_hexapod.png)

## Introduction

In May of 2023 we set out to design a Hexapod (6 legged robot) which would primarily be 3D printed and cost under $500 for parts and materials. This hexapod will eventually be controlled with a wireless controller, have multiple options for maneuverability, and [stretch goal] utilize machine learning to track and follow objects.

We're still in the early stages of this project, as development is slowed down by geographical distance and full time jobs. But we'll get there eventually (and hopefully not too much over budget).

Who are we?

- [Danny Iacobacci](https://www.linkedin.com/in/diacobacci/)
- [Zack Schwid](https://www.linkedin.com/in/zacharyschwid/)
- [Dillon Kane](https://www.linkedin.com/in/dillonkane/)

This project is not yet ready for others to recreate as we're still making significant changes to the code, electronics, and CAD files. But this github repo will always have the most up-to-date information and files.

Once this project is closer to completion we will be leaving detailed documentation on how to build one of your own (but note: it is a very involved project).

## Mk1

This version of the Hexapod will be completed in the summer of 2024. This Hexapod will come in under the $500 budget, utilizing a Raspberry Pi Zero W + Teensy 4.1 for the main command and control interface. While this hexapod was a great proof of concept (it is controllable via a generic bluetooth controller, has one default gait, etc) we quickly realized that this version would be a bottleneck for our ambitions -- both in hardware and software limitations. That's why we decided to end the Mark 1 journey once it is fully walking, and pivot into creating a much feature-full Mk2.

The files for the Mk1 are available, but may be incomplete due to the project pivot. We would encourage anyone wanting to build their own hexapod to wait until we finish Mk2, as we are tracking quite a few limitations in the design. All aspects of the Hexapod are being improved for Mk2, including every 3D printed part and the electronics to be more feature rich.

## Mk1 Project Glossary

- [3D Models](<Mk1/Mk1 3D Prints>)
- [Bill of Materials](<Mk1/Mk1 Bill-of-Materials/BOM.md>)
- [Teensy Code](./HexapodController/)
- [Raspberry Pi Zero Code](<Mk1/Mk1_Hexapod_Command>)
- [Electronics](<./Mk1/Mk1 PCB>)

## Mk2

We're just getting started on planning for Mark2, but some of the details include:

- Raspberry Pi 5
- Switching over to ROS2 on Ubuntu
- Redesigned PCB to include new features
- Improved CAD

For more info on what we're planning to change, checkout our Github issues!

## License

[License](./LICENSE)
