@echo off

set FIRMWARE_PATH= .\submodules\BHy-SensorAPI\firmware
set COINES_BRIDGE_APP30= .\submodules\coines\firmware\app3.0
set COINES_BRIDGE_APP31= .\submodules\coines\firmware\app3.1
set COINES_BRIDGE_NICLA= .\submodules\coines\firmware\nicla
set COINES_APP_SWITCH= .\submodules\coines\tools\app_switch
set COINES_USB_DFU= .\submodules\coines\tools\usb-dfu

:: Make directories for release package
mkdir .\release
mkdir .\release\BHI3-firmwares
mkdir .\release\BHI3-firmwares\BHI360
mkdir .\release\docs
mkdir .\release\PC
mkdir .\release\PC\bin
mkdir .\release\firmware
mkdir .\release\firmware\app30
mkdir .\release\firmware\app31
mkdir .\release\firmware\nicla
mkdir .\release\tools
mkdir .\release\tools\app_switch
mkdir .\release\tools\usb-dfu

:: Create executable folder based on 64-bit compiler (x64) or 32-bit compiler (x86)
gcc -dumpmachine > GCC.txt
FOR /F %%i IN (GCC.txt) DO set GCC= %%i
if not %GCC:x86_64=%==%GCC% (
set COMPILER=x64
) ELSE (
set COMPILER=x86
)

mkdir .\release\PC\bin\%COMPILER%
del GCC.txt

:: Set output folders in variables
set PC_BIN_PATH= .\release\PC\bin\%COMPILER%
set PC_COINES_BRIDGE_APP30= .\release\firmware\app30
set PC_COINES_BRIDGE_APP31= .\release\firmware\app31
set PC_COINES_BRIDGE_NICLA= .\release\firmware\nicla
set PC_COINES_APP_SWITCH= .\release\tools\app_switch
set PC_COINES_USB_DFU= .\release\tools\usb-dfu
set PC_DOCS=.\release\docs

@echo --------------------------------------------------- 
@echo            Updating Firmwares of BHI360 
@echo ---------------------------------------------------

:: Copy BHI360 firmwares
xcopy /s /y %FIRMWARE_PATH%\bhi360\*.fw .\release\BHI3-firmwares\BHI360

@echo ---------------------------------------------------------- 
@echo      Updating COINES tools and COINES_BRIDGE firmwares
@echo ----------------------------------------------------------

:: Copy coines_bridge_app30 folders 
xcopy /s /y %COINES_BRIDGE_APP30% %PC_COINES_BRIDGE_APP30%

:: Copy coines_bridge_app31 folders 
xcopy /s /y %COINES_BRIDGE_APP31% %PC_COINES_BRIDGE_APP31%

:: Copy coines_bridge_nicla folders 
xcopy /s /y %COINES_BRIDGE_NICLA% %PC_COINES_BRIDGE_NICLA%

:: Copy app_switch and usb-dfu 
echo D|xcopy /y %COINES_APP_SWITCH%\*.exe %PC_COINES_APP_SWITCH%
echo D|xcopy /y %COINES_APP_SWITCH%\*.md %PC_COINES_APP_SWITCH%
echo D|xcopy /y %COINES_USB_DFU% %PC_COINES_USB_DFU%

@echo ---------------------------------------------------------- 
@echo                     Update Documents
@echo ----------------------------------------------------------

:: Copy docs folder
xcopy /s /y docs %PC_DOCS%

@echo --------------------------------------------------------------
@echo  Build PC executables for BHyCLI (I2C, SPI) and Decompressor
@echo --------------------------------------------------------------

:: Build executable for BHyCLI for TARGET PC with I2C Interface
mingw32-make COINES_INSTALL_PATH=submodules/coines API_LOCATION=source COMMON_LOCATION=source/common TARGET=PC BHY_INTF=I2C all
move /y bhycli.exe i2c_bhycli.exe

:: Build executable for BHyCLI for TARGET PC with SPI Interface
mingw32-make COINES_INSTALL_PATH=submodules/coines API_LOCATION=source COMMON_LOCATION=source/common TARGET=PC all
move /y bhycli.exe spi_bhycli.exe

:: Copy executable to %PC_BIN_PATH%
xcopy /y .\*bhycli.exe %PC_BIN_PATH%

:: Build executable for decompressor for TARGET PC
cd .\tools\decompressor
mingw32-make COINES_INSTALL_PATH=../../submodules/coines API_LOCATION=../../source COMMON_LOCATION=../../source all

:: Copy decompressor executable to %PC_BIN_PATH%
cd ..\..\
xcopy /y .\tools\decompressor\decompressor.exe %PC_BIN_PATH%
