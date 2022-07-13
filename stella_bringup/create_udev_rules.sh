#!/bin/bash

echo "remap the devices serial port(ttyUSBX, ttySX, videoX) to  ydlidar, WLKATA, AHRS, Motordriver, Bluetooth, C920"
echo "devices usb connection as /dev/YDLIDAR, /dev/WLKATA, /dev/AHRS, /dev/MW, /dev/BT, /dev/C920, check it using the command : ls -l /dev|grep -e ttyUSB -e ttyS -e video"
echo "start copy stella.rules to  /etc/udev/rules.d/"
echo "`rospack find stella_bringup`/stella.rules"
sudo cp `rospack find stella_bringup`/stella.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm trigger
echo "finish "
