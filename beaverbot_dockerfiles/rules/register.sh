sudo cp 99-usb-camera.rules /etc/udev/rules.d/
sudo cp 99-mmal-camera.rules /lib/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger