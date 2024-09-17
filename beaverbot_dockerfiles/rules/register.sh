sudo cp 99-usb-driver.rules /etc/udev/rules.d/
sudo cp 99-usb-imu.rules /etc/udev/rules.d/
sudo cp 99-usb-gps.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger