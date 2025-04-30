
# BHyCLI Manual 📘

## Table of Contents
1. [Compatibility](#compatibility)
2. [Getting Started](#getting-started)
3. [Release Package](#release-package)
4. [Execute BHyCLI Application](#execute-bhycli-application🏃‍♂️)

## Compatibility
| Items      | BHyCLI | FW     | BSX4   |BHy SensorAPI | COINES SDK   | Supported Boards | Supported Sensors |
| :---:      | :---:  | :----: | :----: | :----:       | :----:       | :----: | :----: |
| version    | 0.5.1  | 1.1.18 | IR84.3 | 2.0.0        | 2.10.2       | APP30<br>APP31 | BHI360 |

## Getting Started

## Cloning the Repository with Submodules
To clone this repository along with all its submodules, use the following command:

```sh
git clone --recursive https://github.com/boschsensortec/BHy2CLI.git
```

## Initializing and Updating Submodules
If you have already cloned the repository without submodules, you can initialize and update them using:

```sh
git submodule update --init --recursive
```

## Keeping Submodules Updated
To ensure submodules remain up to date when pulling the latest changes, run:

```sh
git pull --recurse-submodules
```

Then, update the submodules using:

```sh
git submodule update --recursive
```

## Release package

Release package helps to generate executables and binaries of BHyCLI , which can then be used to test BHyCLI Application on PC/MCU in Application Board APP3.0 / APP3.1
### Creating Release package :
```
1.Open powershell in root directory of BHyCLI project.
2.For TARGET PC, run .\scripts\build.bat
3.For TARGET MCU_APP30, run .\scripts\build_app30.bat
4.For TARGET MCU_APP31, run .\scripts\build_app31.bat
```
#### Release folder structure :
		 - BHI3-firmwares/BHI360 : Holds BHI360 firmwares
		 - firmware              : Holds bootloader, coines_bridge and MTP firmwares for APP3.0 and APP3.1
		 - MCU                   : Holds APP3.0 and APP3.1 BHyCLI binaries (I2C, SPI) and batch files to write BHI360 firmwares (present in BHI3-firmwares/BHI360) to External FLASH
		 - PC                    : Holds executables for BHyCLI (I2C, SPI) and decompressor
			 - Executables are present in,
				 - PC/bin/x86 for 32-bit compiler
				 - PC/bin/x64 for 64-bit compiler
		 - tools                 : Holds app_switch and usb-dfu files required to flash coines_bridge firmware (for TARGET PC) and to write BHI360 firmwares to External FLASH (for TARGET MCU_APP30 and MCU_APP31)

## Execute BHyCLI Application🏃‍♂️


- Refer Section 2 of BHyCLI_User_Guide.pdf (From docs/ folder)


