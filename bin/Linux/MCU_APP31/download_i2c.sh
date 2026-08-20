#!/bin/sh

../tools/app_switch/app_switch usb_dfu_bl
dfu-util --device -,108c:ab39 -a FLASH -D i2c_bhy2cli.bin -R
