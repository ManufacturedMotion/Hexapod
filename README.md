# Hexapod

![Manufactured Motion Logo](./Images/hexapod_logo.png)

## Introduction

In May of 2023 we set out to design a Hexapod (6 legged robot) which would primarily be 3D printed and cost under $500 for parts and materials. This hexapod will eventually be controlled with a wireless controller, have multiple options for maneuverability, and [stretch goal] utilize machine learning to track and follow objects.

We're still in the early stages of this project, as development is slowed down by geographical distance and full time jobs. But we'll get there eventually (and hopefully not too much over budget).

Who are we?

- [Danny Iacobacci](https://www.linkedin.com/in/diacobacci/)
- [Zack Schwid](https://www.linkedin.com/in/zacharyschwid/)
- [Dillon Kane](https://www.linkedin.com/in/dillonkane/)

This project is not yet ready for others to recreate as we're still making significant changes to the code, electronics, and CAD files. But this github repo will always have the most up-to-date information and files.

Once this project is closer to completion we will be leaving detailed documentation on how to build one of your own (but note: it is a very involved project).

## Mark1

This version of the Hexapod is mostly functional as of June 2024. This Hexapod came in under the $500 budget, utilizing a Raspberry Pi Zero W + Teensy 4.1 for the main command and control interface. While this hexapod was a great proof of concept, we quickly realized that this version would be a bottleneck for our ambitions -- both in hardware and software limitations. That's why we decided to end the Mk1 journey with it is fully walking, and completely revamp the Hexapod into a much feature-full Mk2. The Mk2 is expected to be the definitive version of the Hexapod, with smaller revisions under the Mk2 umbrella.  

The files for the Mk1 will be available shortly, but we would encourage anyone wanting to repro this project to wait until we finish Mk2, as we are tracking quite a few limitations in this design. Including issues with pairing controllers, limited gait and motions, and overall it is not a polished design.

## Mk1 Project Glossary

- [3D Models](<Mark1/Mark1 3D Prints>)
- [Bill of Materials](<Mark1/Mark1 Bill-of-Materials/BOM.md>)
- [Teensy Code](./HexapodController/)
- [Raspberry Pi Zero Code](<Mark1/Mark1 RPi Zero W>)
- [Electronics](<./Mark1/Mark1 PCB>)

## Mk2

We're just getting started on planning for Mark2, but some of the details include:

- Raspberry Pi 5
- Switching over to ROS2 on Ubuntu
- Redesigned PCB to include new features
- Improved CAD

For more info on what we're planning to change, checkout our Github issues!

## Coding Standards

We primarily follow [Google's C++ coding standards](https://google.github.io/styleguide/cppguide.html#Function_Names) with the exceptions and highlights below:

### Naming Conventions

- Class Names: ClassName
- Constant: kVariableName
- Defined: VARIABLENAME
- File Names: file_name.extension
- Function Names: functionName
- Private Class: _class
- Type Names: TypeName
- Variable Names: snake_case
- Struct: StructName

- Variable names should be self-documenting (I.E., `LegSegment1` instead of `A`)

### Misc Standards

- Every file should have exactly 1 empty line as the last line.
- There should not be any spaces at the end of a line.

## License

[License](./LICENSE)
