#!/bin/sh
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0005", MODE="0666", SYMLINK+="ttyVIG0"' >> /etc/udev/rules.d/99-usb-serial.rules
