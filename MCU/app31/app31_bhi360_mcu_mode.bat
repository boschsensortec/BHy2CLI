@echo off

set DFU_UTIL= .\..\..\tools\usb-dfu\dfu-util.exe
set APP_SWITCH= .\..\..\tools\app_switch\app_switch.exe
set MTP_SENDFILE=.\mtp-tools\mtp-sendfile.exe
set MTP_DELETE=.\mtp-tools\mtp-delfile.exe
set MTP_GETFILE=.\mtp-tools\mtp-connect.exe --getfile
set MTP_FORMAT=.\mtp-tools\mtp-format.exe
set BHI3_FILES_DIR=.\..\..\BHI3-firmwares\BHI360

:: Load the bhycli firmware (SPI Interface added by default. To use I2C interface, change to 'i2c_bhycli.bin')
%APP_SWITCH% usb_dfu_bl
%DFU_UTIL%  --device -,108c:ab39 -a FLASH -D .\spi_bhycli.bin -R

:: Sleep for 5s
timeout /t 5 /nobreak > NUL

echo ---------------------------------------------
:: Switch to Bootloader mode
echo Writing files to the external flash
%APP_SWITCH% usb_dfu_bl > NUL

:: Load and run MTP firmware in RAM
%DFU_UTIL% --device -,108c:ab39 -a RAM -D .\usb_mtp_WinUSB_RAM.bin -R > NUL 2> NUL
if NOT %ERRORLEVEL%==0 ( 
echo Unable to load MTP firmware. Error Code is: %ERRORLEVEL%
echo Please try unplugging and re-inserting USB cables
pause
exit 2
)

:: Sleep for 5s after firmware upload
timeout /t 5 /nobreak > NUL

for /f %%i in ('dir %BHI3_FILES_DIR% /b') do (
echo - Deleting %%i if present
%MTP_DELETE% -f %%i  > NUL 2> NUL
echo + Writing %%i
%MTP_SENDFILE% %BHI3_FILES_DIR%\%%i %%i  > NUL 2> NUL
)

:: Sleep for 2s after file upload
timeout /t 2 /nobreak > NUL

echo ---------------------------------------------
echo Reset or restart the board to run the bhycli firmware
pause
