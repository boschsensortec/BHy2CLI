
# BHy2CLI Manual 📘

## Table of Contents
1. [Compatibility](#compatibility)
2. [Folder Structure](#folder-structure-🗂️)
3. [Getting Started](#getting-started)
4. [Release Package](#release-package)
5. [Execute BHy2CLI Application](#execute-BHy2cli-application🏃‍♂️)

## Compatibility
| Items      | BHy2CLI | FW     | BSX   |BHI SensorAPI | COINES SDK   | Supported Boards | Supported Sensors |
| :---:      | :---:   | :----: | :----: | :----:        | :----:      | :----: | :----:  |
| version    | 1.0.0   | 1.1.18<br>1.0.0.4 | IR84.3<br>1.3.2 | 2.2.0<br>1.0.0         | 2.10        | APP30<br>APP31   | BHI360<br>BHI385     |

## Folder Structure 🗂️

The BHy2CLI repositories follow this folder structure:

```shell
BHy2CLI:
    +---bin
    +---docs
    +---scripts
    +---source
    +---submodules
    +---tools
    +---Makefile
    +---README.md
```

* bin: Contains BHy2CLI executable files
* docs: Contains release documents (User_Guide, CHANGELOG.md, Compatibility.txt, ...)
* scripts: Contains batch files to clean, generate and download BHy2CLI executables/binaries 
* source: Holds the BHy2CLI source files.
* submodules: Destination folder for package dependency modules.
* tools: Contains initial automation tools.
* Makefile: A script to configure and build BHy2CLI
* README.md: A text file that introduces and explains a software

## Getting Started

To clone BHy2CLI from Github, follow the below command

```
1. Open your terminal or Git Bash on your local machine.
2. Use the cd command to navigate to the folder where you want to save the project.
3. Type the following command 
  git clone --recurse-submodules https://github.com/boschsensortec/BHy2CLI.git
4. Press Enter, the repository will download to your computer
```


> ⚠️ **Note**
>
> Ensure you have the necessary access rights to the repository.
> If you encounter any errors related to repository access, please verify your SSH keys and user permissions.

### Steps to include a new sensor API (Optional)
1. Copy the sensor API folder under "submodules" so that the API source code is available under "submodules/bhixxx".
By default, the source code and Makefile locate the BHI360 sensor API v2.2.0 under "submodules/bhi360".
2. Update the Makefile of the CLI to include definitions and source code for the new sensor API. Follow these steps:
   - Add the location of the new sensor API:
     ```
     CLIxxx_API_LOCATION ?= submodules/bhixxx
     ```
   - Add the source files of the new sensor API to the build process:
     ```
     CLIxxx_API_SRCS := $(wildcard $(CLIxxx_API_LOCATION)/*.c)
     ```
   - Include the source files in the list of C source files:
     ```
     C_SRCS += \
     $(CLIxxx_API_SRCS) \
     ```
   - Add the API location to the include paths:
     ```
     INCLUDEPATHS += . \
     $(CLIxxx_API_LOCATION) \
     ```
3. In "bhy_defs.c", locate the `all_sensor_api_entry` array definition. Add the new api_entry inside. This ensures the CLI can recognize and use the new sensor API. For example:
   ```c
   #include "bhixxx_api_entry.h"
   #include "bhixxx_defs.h"
   ```

   ```c
   static const ChipAPIEntry all_sensor_api_entry[] = {
    ..., { BHIxxx_CHIP_ID, bhixxx_sensor_api_entry }, { 0, NULL } /* End of
                                                                   * table
                                                                   * marker */
   };
   ```

## Release package

Release package helps to generate executables and binaries of BHy2CLI , which can then be used to test BHy2CLI Application on PC/MCU in Application Board APP3.0 / APP3.1
### Creating Release package :
```
1. Open powershell in root directory of BHy2CLI project.
2. For TARGET PC, run .\scripts\build.bat
3. For TARGET MCU_APP30, run .\scripts\build_app30.bat
4. For TARGET MCU_APP31, run .\scripts\build_app31.bat
```
#### Release folder structure :
		 - BHI3-firmwares/BHI360 : Holds BHI360 firmwares
		 - firmware              : Holds bootloader, coines_bridge and MTP firmwares for APP3.0 and APP3.1
		 - MCU                   : Holds APP3.0 and APP3.1 BHy2CLI binaries (I2C, SPI) and batch files to write BHI360 firmwares (present in BHI3-firmwares/BHI360) to External FLASH
		 - PC                    : Holds executables for BHy2CLI (I2C, SPI) and decompressor
			 - Executables are present in,
				 - PC/bin/x86 for 32-bit compiler
				 - PC/bin/x64 for 64-bit compiler
		 - tools                 : Holds app_switch and usb-dfu files required to flash coines_bridge firmware (for TARGET PC) and to write BHI360 firmwares to External FLASH (for TARGET MCU_APP30 and MCU_APP31)

## Execute BHy2CLI Application🏃‍♂️


- Refer Section 2 of BHy2CLI_User_Guide.pdf (From docs/ folder)


> ⚠️ **Note**
>
> If the TARGET is MCU_APP30 or MCU_APP31, please copy the Firmware into Application board memory.
```
1. Switch application board to MTP mode.
2. Copy Firmware from Window to application board memory
```
> Ensure the file name length of Firmware is not more than 39 characters, then please short its name if necessary.