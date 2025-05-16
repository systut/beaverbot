sudo cp 99-usb-*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger