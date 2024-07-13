# Hexapod

![Manufactured Motion Logo](./Images/hexapod_logo.png)

## Introduction

In May of 2023 we set out to design a Hexapod (6 legged robot) which would primarily be 3D printed and cost under $500 for parts and materials. This hexapod will eventually be controlled with a wireless controller, have multiple options for maneuverability, and [stretch goal] utilize machine learning to track and follow objects.

We're still in the early stages of this project, as development is slowed down by geographical distance and full time jobs. But we'll get there eventually (and hopefully not too much over budget).

Who are we?

- [Danny Iacobacci](https://www.linkedin.com/in/diacobacci/)
- [Zack Schwid](https://www.linkedin.com/in/zacharyschwid/)
- [Dillon Kane](https://www.linkedin.com/in/dillonkane/)

## Getting Started

This project is not yet ready for others to recreate as we're still making significant changes to the code, electronics, and CAD files. But this github repo will always have the most up-to-date information and files.

Once this project is closer to completion we will be leaving detailed documentation on how to build one of your own (but note: it is a very involved project).

## Project Glossary 

- [3D Models](./Finalized%203D%20Printed%20Parts/)
- [Bill of Materials](./Bill-of-Materials/BOM.md)
- [Code](./HexapodController/)
- [Electronics](./PCB/)

## Coding Standards

We follow [Google's C++ coding standards](https://google.github.io/styleguide/cppguide.html#Function_Names) with the exceptions and highlights below

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

- Every file should have exactly 1 empty line as the last line
- There should not be any spaces at the end of a line

## License

[License](./LICENSE)
