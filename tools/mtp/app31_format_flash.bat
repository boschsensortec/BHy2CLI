@echo off

set DFU_UTIL= .\..\..\tools\usb-dfu\dfu-util.exe
set APP_SWITCH= .\..\..\tools\app_switch\app_switch.exe
set MTP_SENDFILE=.\mtp-tools\mtp-sendfile.exe
set MTP_GETFILE=.\mtp-tools\mtp-connect.exe --getfile
set MTP_FORMAT=.\mtp-tools\mtp-format.exe

:: Load the USB-Bridge firmware
%APP_SWITCH% usb_dfu_bl
%DFU_UTIL% --device -,108c:ab39 -a RAM -D .\usb_mtp.pkg -R

:: Sleep for 5s
timeout /t 5 /nobreak > NUL

echo ---------------------------------------------
echo Reset or restart the board to format the external flash
pause

:: Switch to Bootloader mode
%APP_SWITCH% usb_dfu_bl > NUL

:: Load and run MTP firmware in RAM
%DFU_UTIL% --device -,108c:ab39 -a RAM -D .\usb_mtp_WinUSB_RAM.bin -R > NUL 2> NUL
if NOT %ERRORLEVEL%==0 ( 
echo Unable to load MTP firmware. Error Code is: %ERRORLEVEL%
echo Please try unplugging and re-inserting USB cables
pause
exit 2
)

:: Sleep for 2s after firmware upload
timeout /t 2 /nobreak > NUL
%MTP_FORMAT% -y


echo ---------------------------------------------
echo Reset or restart the board to run the pre-existing application
pause
