@echo off

set FIRMWARE_PATH= .\submodules\BHy-SensorAPI\firmware
set COINES_APP_SWITCH= .\submodules\coines\tools\app_switch
set COINES_USB_DFU= .\submodules\coines\tools\usb-dfu
set MTP= .\tools\mtp
set MTP_TOOLS= .\tools\mtp\mtp-tools
set USB_MTP= .\submodules\coines\firmware\app3.0\mtp_fw_update
set COINES_BRIDGE_APP30= .\submodules\coines\firmware\app3.0
set COINES_BRIDGE_NICLA= .\submodules\coines\firmware\nicla

:: Make directories for release package
mkdir .\release
mkdir .\release\BHI3-firmwares
mkdir .\release\BHI3-firmwares\BHI360
mkdir .\release\docs
mkdir .\release\firmware
mkdir .\release\firmware\app30
mkdir .\release\firmware\nicla
mkdir .\release\MCU
mkdir .\release\MCU\app30
mkdir .\release\MCU\app30\mtp-tools
mkdir .\release\tools
mkdir .\release\tools\app_switch
mkdir .\release\tools\usb-dfu

set MCU_COINES_BRIDGE_APP30= .\release\firmware\app30
set MCU_COINES_BRIDGE_NICLA= .\release\firmware\nicla
set MCU_BIN_PATH= .\release\MCU\app30
set MCU_MTP_TOOLS= .\release\MCU\app30\mtp-tools
set MCU_COINES_APP_SWITCH= .\release\tools\app_switch
set MCU_COINES_USB_DFU= .\release\tools\usb-dfu
set MCU_DOCS=.\release\docs

@echo --------------------------------------------------- 
@echo            Updating Firmwares of BHI360
@echo ---------------------------------------------------

:: Copy BHI360 firmwares
xcopy /s /y %FIRMWARE_PATH%\bhi360\*.fw .\release\BHI3-firmwares\BHI360

@echo ---------------------------------------------------------- 
@echo      Updating COINES tools and COINES APP30 firmwares
@echo ----------------------------------------------------------

:: Copy coines_bridge_app30 folders 
xcopy /s /y %COINES_BRIDGE_APP30% %MCU_COINES_BRIDGE_APP30%

:: Copy coines_bridge_nicla folders 
xcopy /s /y %COINES_BRIDGE_NICLA% %MCU_COINES_BRIDGE_NICLA%

:: Copy app_switch and usb-dfu 
echo D|xcopy /y %COINES_APP_SWITCH%\*.exe %MCU_COINES_APP_SWITCH%
echo D|xcopy /y %COINES_APP_SWITCH%\*.md %MCU_COINES_APP_SWITCH%
echo D|xcopy /y %COINES_USB_DFU% %MCU_COINES_USB_DFU%

@echo ---------------------------------------------------------- 
@echo                   Updating MTP tools
@echo ----------------------------------------------------------

xcopy /y %MTP_TOOLS% %MCU_MTP_TOOLS%
xcopy /y %MTP%\app30_bhi360*.bat %MCU_BIN_PATH%
xcopy /y %MTP%\app30_format_flash.bat %MCU_BIN_PATH%
xcopy /y %USB_MTP%\usb_mtp_WinUSB_RAM.bin %MCU_BIN_PATH%
xcopy /y %USB_MTP%\usb_mtp.pkg %MCU_BIN_PATH%

@echo ---------------------------------------------------------- 
@echo                     Update Documents
@echo ----------------------------------------------------------

:: Copy docs folder
xcopy /s /y docs %MCU_DOCS%

@echo ---------------------------------------------------------- 
@echo        Build MCU_APP30 binary for BHyCLI (I2C, SPI) 
@echo ----------------------------------------------------------

:: Build binary for BHyCLI for TARGET MCU_APP30 with I2C Interface
mingw32-make COINES_INSTALL_PATH=submodules/coines API_LOCATION=source COMMON_LOCATION=source/common TARGET=MCU_APP30 BHY_INTF=I2C LOCATION=FLASH all
move /y bhycli.bin i2c_bhycli.bin

:: Build binary for BHyCLI for TARGET MCU_APP30 with SPI Interface
mingw32-make COINES_INSTALL_PATH=submodules/coines API_LOCATION=source COMMON_LOCATION=source/common TARGET=MCU_APP30 LOCATION=FLASH all
move /y bhycli.bin spi_bhycli.bin

:: Copy binary to %MCU_BIN_PATH%
xcopy /y .\*bhycli.bin %MCU_BIN_PATH%
