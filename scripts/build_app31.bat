::Copyright (c) 2026 Bosch Sensortec GmbH. All rights reserved.

::BSD-3-Clause

::Redistribution and use in source and binary forms, with or without
::modification, are permitted provided that the following conditions are met:

::1. Redistributions of source code must retain the above copyright
::   notice, this list of conditions and the following disclaimer.

::2. Redistributions in binary form must reproduce the above copyright
::    notice, this list of conditions and the following disclaimer in the
::    documentation and/or other materials provided with the distribution.
::3. Neither the name of the copyright holder nor the names of its
::    contributors may be used to endorse or promote products derived from
::    this software without specific prior written permission.

::THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
::"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
::LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
::FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
::COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
::INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
::(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
::SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
::HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
::STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
::IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
::POSSIBILITY OF SUCH DAMAGE.


@echo off

set FIRMWARE_PATH_BHI360= .\submodules\bhi360\firmware
set FIRMWARE_PATH_BHI385= .\submodules\bhi385\firmware
set COINES_APP_SWITCH= .\submodules\coines\tools\app_switch
set COINES_USB_DFU= .\submodules\coines\tools\usb-dfu
set MTP= .\tools\mtp
set MTP_TOOLS= .\tools\mtp\mtp-tools
set USB_MTP= .\submodules\coines\firmware\app3.1\mtp_fw_update
set COINES_BRIDGE_APP31= .\submodules\coines\firmware\app3.1

:: Make directories for release package
mkdir .\release
mkdir .\release\BHI3-firmwares
mkdir .\release\BHI3-firmwares\BHI360
mkdir .\release\BHI3-firmwares\BHI385
mkdir .\release\docs
mkdir .\release\firmware
mkdir .\release\firmware\app31
mkdir .\release\MCU
mkdir .\release\MCU\app31
mkdir .\release\MCU\app31\mtp-tools
mkdir .\release\tools
mkdir .\release\tools\app_switch
mkdir .\release\tools\usb-dfu

set MCU_COINES_BRIDGE_APP31= .\release\firmware\app31
set MCU_BIN_PATH= .\release\MCU\app31
set MCU_MTP_TOOLS= .\release\MCU\app31\mtp-tools
set MCU_COINES_APP_SWITCH= .\release\tools\app_switch
set MCU_COINES_USB_DFU= .\release\tools\usb-dfu
set MCU_DOCS=.\release\docs

@echo -------------------------------------------------------------
@echo            Updating Firmwares of BHI360 and BHI385
@echo -------------------------------------------------------------

:: Copy BHI360 firmwares
xcopy /s /y %FIRMWARE_PATH_BHI360%\bhi360\*.fw .\release\BHI3-firmwares\BHI360

:: Remove BHI360 folder if no FW
dir /b /s /a "release\BHI3-firmwares\BHI360" | findstr .>nul || (
  rmdir /s /q release\BHI3-firmwares\BHI360
)

:: Copy BHI385 firmwares
xcopy /s /y %FIRMWARE_PATH_BHI385%\bhi385\*.fw .\release\BHI3-firmwares\BHI385

:: Remove BHI385 folder if no FW
dir /b /s /a "release\BHI3-firmwares\BHI385" | findstr .>nul || (
  rmdir /s /q release\BHI3-firmwares\BHI385
)

@echo ---------------------------------------------------------- 
@echo      Updating COINES tools and COINES APP31 firmwares
@echo ----------------------------------------------------------

:: Copy coines_bridge_app31 folders 
xcopy /s /y %COINES_BRIDGE_APP31% %MCU_COINES_BRIDGE_APP31%

:: Copy app_switch and usb-dfu 
echo D|xcopy /y %COINES_APP_SWITCH%\*.exe %MCU_COINES_APP_SWITCH%
echo D|xcopy /y %COINES_APP_SWITCH%\*.md %MCU_COINES_APP_SWITCH%
echo D|xcopy /y %COINES_USB_DFU% %MCU_COINES_USB_DFU%

@echo ---------------------------------------------------------- 
@echo                   Updating MTP tools
@echo ----------------------------------------------------------

xcopy /y %MTP_TOOLS% %MCU_MTP_TOOLS%
if exist "release\BHI3-firmwares\BHI360" (
  xcopy /y %MTP%\app31_bhi360*.bat %MCU_BIN_PATH%
)
if exist "release\BHI3-firmwares\BHI385" (
  xcopy /y %MTP%\app31_bhi385*.bat %MCU_BIN_PATH%
)
xcopy /y %MTP%\app31_format_flash.bat %MCU_BIN_PATH%
xcopy /y %USB_MTP%\usb_mtp_WinUSB_RAM.bin %MCU_BIN_PATH%
xcopy /y %USB_MTP%\usb_mtp.pkg %MCU_BIN_PATH%

@echo ---------------------------------------------------------- 
@echo                     Update Documents
@echo ----------------------------------------------------------

:: Copy docs folder
xcopy /s /y docs %MCU_DOCS%

@echo ---------------------------------------------------------- 
@echo        Build MCU_APP31 binary for BHy2CLI (I2C, SPI) 
@echo ----------------------------------------------------------

:: Build binary for BHy2CLI for TARGET MCU_APP31 with I2C Interface
mingw32-make COINES_INSTALL_PATH=submodules/coines API_LOCATION=source COMMON_LOCATION=source/common TARGET=MCU_APP31 BHY_INTF=I2C LOCATION=FLASH all
move /y bhy2cli.bin i2c_bhy2cli.bin

:: Build binary for BHy2CLI for TARGET MCU_APP31 with SPI Interface
mingw32-make COINES_INSTALL_PATH=submodules/coines API_LOCATION=source COMMON_LOCATION=source/common TARGET=MCU_APP31 LOCATION=FLASH all
move /y bhy2cli.bin spi_bhy2cli.bin

:: Copy binary to %MCU_BIN_PATH%
xcopy /y .\*bhy2cli.bin %MCU_BIN_PATH%
