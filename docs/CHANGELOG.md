# Changelog

# v0.5.1
 - Features added:
 1. Added command to log and stream the data together.
 2. Added command to save and load BSX parameters.
 3. Added command to set physical sensor range. (command: phyrangeconf)
 4. Added command to connect to user selectable COM port. (PC mode only)
 5. Added support for downsampling when activating sensor.
 6. Following commands are added newly for configuring various parameters
	
	- set & get virtual sensor configuration
	setvirtsenconf
	getvirtsenconf
	
	- to set swim logging
	swimsetlogging

	- set & get accelerometer axis remapping
	accsetar
	accgetar

	- trigger & get accelerometer NVM status
	acctrignvm
	accgetnvm
	
	- to trigger & get gyroscope NVM status
	gyrotrignvm
	gyrogetnvm
	
	- set & get maganetometer power mode
	magsetpwm
	maggetpwm
	
	- set & get barometer pressure
	baro1setcnfg
	baro1getcnfg
	baro2setcnfg
	baro2getcnfg
	
	- set & get step counter configuration
	scsetcnfg
	scgetcnfg
	
	- set & get misallignment mode
	hmcsetmode
	hmcgetmode
	
	- trigger foc for physical sensors
	foc

	- to get chip id of the shuttle
	chipid

	- set & get system parameters
	syssetphyseninfo
	sysgetphysenlist
	sysgetvirsenlist
	sysgettimestamps
	sysgetfwversion
	sysgetfifoctrl
	syssetwkffctrl
	syssetnwkffctrl
	sysgetmectrl
	syssetmectrl

	- set & get bsec parameters
	bsecsetalstate
	bsecgetalstate
	bsecsettempoff
	bsecgettempoff
	bsecsetsamrate
	bsecgetsamrate

	- set & get activity configuration
	sethearactvcnfg
	gethearactvcnfg
	setwearactvcnfg
	getwearactvcnfg
	
	- get virtual sensor parameters
	virtseinfo

	- set & get bsx parameters
	setbsxparam
	getbsxparam
	getbsxver
	logandstream
	
	- set physical range configuration
	phyrangeconf


- Bugs Fixed: 
1. Bug Fixed: PC mode current rises to 140 uA after disabling the sensors.
2. Bug Fixed: Multi-tap were not correctly identified.
3. Bug Fixed: Scaling factor calculation, now, takes account the dynamic range instead of default range.
4. Bug Fixed: Fixed default scaling factor for different Magnetometer variants
5. Bug Fixed: Incorrect default scaling factor for BMP Temperature sensor

- Added BHI3 variant latest version Firmware Images

- Decompressor Version v1.0.0
Features Added:
	1. Label is now added to all rows instead of only one single row. IMPROVEMENT
	2. Version number added.
Bugs Fixed:
	1. Label column position is fixed. It was printing at wrong column.

# v0.4.11
- Resolved decompressing issue for logged data.

# v0.4.10-rc1
- Resolved BLE communication failure issue while triggering FOC.
- Resolved accuaracy issue in the logged data for 'head orientation' virtual sensors.
- Resolved blank column issue in the logged data for 'head orientation' virtual sensors.

# v0.4.9
- Added support HEX streaming mode.
- Added support for listing the schema information of the loaded sensors
- Added support for Head Orientation virtual sensors.
- Added support for Head Orientation Parameter Configuration.
- Removed the support for PDR
- Rectified parsing callback to s16_to_float for Temperature Sensor

# v0.4.8
- Optimised the Data Injection feature for generic use by removing Sensor ID dependnecy
- Integrated generic file parser for txt/bin files and resolved length dependency, for Data Injection
- Resolved Data Injection Issue in PC Mode
- Resolved EOF check failing for Data Injection in PC Mode
- Changed the Pattern Size check condtion for Klio, to accomadate loading of Adaptive Patterns
- Added notification for File Transfer status for 'wrfile' command
- Added support for getting the list of active sensors along with their configurations and currently open log file
- Added support for retrieving Post Mortem data (Current supported only for MCU mode)
- Updated the coines_bridge firmware in app30-firmware folder.
- Added requisite Firmwares for validation of Data Injection and Post Mortem features.
- Added support for BHI3 Sensor API
- Added support for new sensors -
	- Multi-Tap Detector
	- Activity recognition for wearables
	- No Motion
	- Wrist wear wake-up
	- Wrist gesture detector
- Added support for configuring the Physical Sensor Control Parameters
- Added BHI3 variant Firmware Images
- Added support for preparing the board for BHI3 Sensor
- Increase BLE Transmission Power to +8dBM
- Removed all the unsigned Firmware Images and updated all the Firmware Images for BHI360, BHI380 and BHI260AP to the latest versions
- Added support getting Physical Sensor Information.
- Updated the Wrist Gesture output as per new firmware.
- Added range limit details for ACC/Gyro FOC

# v0.4.7
- Integrated head orientation feature to bhy2cli example
- Updated firmware images for head orientation
- Integrated PDRlog feature to bhy2cli example
- Updated firmware images for PDR
- Corrected the output format for Klio_Log sensor
- Removed '_aux' from BMM150 dependent files owing to filename size dependency.
- Set COINES_BRIDGE as default for PC mode.
- Removed Development Desktop dependent files from app30-firmware folder.
- Updated the bootloader dependent files in app30-firmware folder.
- Updated the coines_bridge firmware in app30-firmware folder.
- Removed check for MCU mode in examples/common.c/close_interface api().
- Erasing Flash when running batch file in order to remove previous loaded application.
- Fixed the lint issues
- Branched COINES submodule to master branch. 
- Updated the connection interval to 15ms from 37.5 ms
- Added support for Deactivating and Listing all the active sensors
- Added support for Read/Write file over BLE/USB
- Extended support for '\n' as a string termination parameter
- Added error handling for file operation commands

# v0.4.6
- Fixed a bug in setting the PDR reference heading
- Fixed typos, most Lint and compiler warnings
- Fixed a bug in the PC build where Ctrl+C didn't exit immediately
- Added legacy command support and shortened CLI command string length from 32 to 16
- Migrated bhy2cli.exe to COINES Bridge beta for better performance
- Updated generic parsing to send the Hex string without any spaces
- Added run-time board recognition to select the correct GPIOs
- Updated close_interfaces() to include bus deconfig
- Cleaned up interrupt config in common and corrected the configuration in the reset function
- Added a first run check with activating a sensor (actse, logse) to query the available sensors from the BHy260, avoids having to call the info command
- Fixed a bug with debug message parsing
- Added dynamic switching support between COM port and BLE, with higher priority to BLE for the MCU bhy2cli
- Added Head tracking and BSEC output frame parsing support 
- Added reset cause code for Reset event 
- Added COINES multi support
- Added New Virtual and Physical sensor IDs
- Updated app30-firmware
- Updated firmware images

# v0.4.5
- Fixed a bug where the internal look up table depended on the info command to be called resulting in an erroneous parsing of sensor data
- Fixed a typo in the addse command's help
- Reverted vt100 support
- Updated to latest COINES master
- Made a specific change in COINES to increase BLE throughput
- Added a new command strbuf that helps buffering data to stream over BLE allowing for higher ODR streaming
- Fixed a bug with out-of-bound array index access
- Added new commands swimver, swimsetfreq, swimgetfreq, swimsetaxes, swimgetaxes, kdisapatt
- Added support for new Klio logging virtual sensor

# v0.4.4
- Fixed a bug where the Wake up Meta events callback was not linked
- Added a clear screen to the initial boot output for the MCU_APP30 target
- Switched COINES submodule to the latest master branch
- Added changes to the sensor API to make it easier to check if a sensor is available
- Changed checking for valid firmware to reading the Feature status rather than Kernel version
- Reduced prints of the bhy2cli PC infos when loading a firmware effectively speeding up loading time
- Added support for Swim
- Refactored some code
- Fixed an issue where the initial accuracy reported a junk value
- Updated to the latest COINES master that added support for unique names for each board
- Fixed a bug where the Green LED didn't flash for the logtxt command
- Fixed an issue where the formatting of the info and ls commands was not uniform
- Completed support for logse (binary logging) by updating the logging format and creating a decompressor
- Deprecated the logtxt command
- Added a command to erase the flash descriptor
- Added more information in the info command
- Moved the cls callback into the common callbacks
- Added support for heartbeat message. The heartbeat message being, '[H]<mcu timestamp ms><\r><\n>
- Moved updating the virtual sensor list from actse to info to reduce actse execution time

# v0.4.3
- Added more Klio commands
- Added new command for version
- Fixed scaling value for PDR outputs

# v0.4.2
- Added PDR commands. 
- Fixed an issue where verbosity worked incorrectly for the PC.
- Updated COINES submodule to latest master

# v0.4.0-beta
 - Fixed verbosity not working as expected
 - Fixed verbose level not reported correctly. Also set default verbose to 0 or no verbose outputs, i.e., no warning or info messages
 - Removed unwanted commands from the PC build
 - Added build option switch for I2C support

# v0.3.0
 - Started to baseline changes.
 - Fixed an issue where disabling the sensor didn't work as expected.