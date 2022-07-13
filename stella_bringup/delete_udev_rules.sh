#!/bin/bash

echo "delete remap the devices serial port(ttyUSBX,ttySX,videoX) to  ydlidar, WLKATA, AHRS, Motordriver, Bluetooth, C920"
echo "sudo rm   /etc/udev/rules.d/stella.rules"
sudo rm   /etc/udev/rules.d/stella.rules
echo " "
echo "Restarting udev"
echo ""
sudo udevadm trigger
echo "finish  delete"
