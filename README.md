# Beaverbot
A ROS workspace contains essential packages for running Beaverbot with various hardwares. 

This workspace is partially written by TUT-Systems Engneering Labroratory members and being used only for research purpores.

<img src=".github/images/beaverbot.jpg" alt="beaverbot"  width="600"/>

### **Table of contents**
- [Beaverbot](#beaverbot)
  - [Hardware Configuration](#hardware-configuration)
    - [GNSS device](#gnss-device)
    - [IMU sensor](#imu-sensor)
    - [LiDAR](#lidar)
    - [Robot control system](#robot-control-system)
  - [Software configuration](#software-configuration)
    - [Localization & Mapping](#localization--mapping)
    - [Path planner](#path-planner)
    - [Path following controller](#path-following-controller)
  - [Manual for running beaverbot](#manual-for-running-beaverbot)
    - [Cloning the repository](#cloning-the-repository)
    - [Setting up the environment](#setting-up-the-environment)
    - [Starting up the system with hardware](#starting-up-the-system-with-hardware)
    - [Scenario 1: Feedforward control](#scenario-1-feedforward-control)
    - [Scenario 2: Feedback control with pure pursuit controller](#scenario-2-feedback-control-with-pure-pursuit-controller)
    - [USB Device Setup for Windows](#usb-device-setup-for-windows)
      - [Installation Steps](#installation-steps)
      - [USB Device Management](#usb-device-management)


## Hardware Configuration

![hardware configuration](.github/images/beaverbot_hardware_configuration.jpg)


### GNSS device
A Global Navigation Satellite System (GNSS) is a satellite configuration, or constellation, that provides coded satellite signals which are processed by a GNSS receiver to calculate position. 

GNSS system is typically only able to provide a positioning accuracy on the order of several of meters. To get better accuracy, [Ichimill service ](https://www.softbank.jp/biz/services/analytics/ichimill/) provided by SoftBank have been chosen for this project. Simly put, base receivers, which have fixed-location, computes correction information and distribute it via Internet, then it is used by our receiver to correct position obtained from GNSS signal. This system is expected to have a accuracy on the order of several of centimeters.

Since the below GNSS receiver which outputs corrected data is used in this package, [nmea_serial_driver](http://wiki.ros.org/nmea_navsat_driver#:~:text=provides%20velocity%20information.-,nmea_serial_driver,-NMEA%20GPS%20Serial) node is good enough for converting receiver's output (NMEA sentences) into ROS messages (`sensor_msgs/NavSatFix`).

> *In case of using different RTK-GNSS receiver, a package contains nodes for receiving correction information from Ichimill service and sending it to receivers is required. Unofficial package [f9p_ichimill](https://github.com/terakenxx/f9p_ichimill) satisfies this requirement.*

![Softbank GNSS receiver](.github/images/ichimill_gnss_receiver.jpg)

### IMU sensor

An Intertial Measurement Unit (IMU) measures and reports body's specific force (also call proper acceleration), angular rate, magnetic fields using a combination of accelerometers, gycroscopes, and magnetomerters.

<img src=".github/images/9axis_imu.png" alt="rt_imu"  width="400"/>

[RT_USB_9axisIMU](https://rt-net.jp/products/usb9imu/) sensor is used in this project. IMU data is obtained via USB port, and expressed in ROS by using [rt_usb_9axisimu_driver](https://github.com/rt-net/rt_usb_9axisimu_driver/tree/noetic-devel?tab=readme-ov-file)

> *The bias errors of IMU components should be calibrated manually for better accuracy. Refer to this [manual](https://github.com/rt-net/RT-USB-9AXIS-00/blob/master/manual/USB%E5%87%BA%E5%8A%9B9%E8%BB%B8IMU%E3%82%BB%E3%83%B3%E3%82%B5ver2.0%E3%83%9E%E3%83%8B%E3%83%A5%E3%82%A2%E3%83%AB.pdf) for more information.*

### LiDAR 

*(in progress)*

### Robot control system 

Tractor part of Beaverbot is an autonomous mobile robot (AMR) whose movement is based on two separately driven wheels placed on either side of the robot body. Velocity commands could be sent to robot via the interface device using serial communication.

 [robot_communication](src/beaverbot_communication/src/beaverbot_communication/robot_communication.cpp) node in [beaverbot_communication](src/beaverbot_communication/) package is responsible for commucating with the robot. The velocity command published on `/cmd_vel` topic will be sent to the robot via interface devices. Concurrently, the rotary encoder data and battery's condition also be published respectively into `/encoder` and `/battery` topics.


## Software configuration 

![software configuration](.github/images/beaverbot_software_configuration.jpg)

### Localization & Mapping

*(in progress)*

### Path planner 

*(in progress)*

### Path following controller 

*(in progress)*

## Manual for running beaverbot 

### Cloning the repository

-----

* **Clone the repository and initialize submodules**

  ```bash
  # Clone the repository
  git clone https://github.com/systut/beaverbot.git
  cd beaverbot

  # Initialize and update submodules
  git submodule update --init
  ```

  This will download the following submodules:
  - `beaverbot_sensors/f9p_ichimill` (Ver0.3.0.0) - For F9P GPS receiver
  - `beaverbot_sensors/rplidar_ros` (2.1.5) - For RPLIDAR
  - `beaverbot_sensors/rt_usb_9axisimu_driver` (1.0.1) - For RT-USB-9AXIS-IMU

### Setting up the environment

-----

* **Enable GUI within Docker containers**

  > **! Caution:** This method exposes PC to external source. Therefore, a more secure alternative way is expected for using GUI within Docker containers. This problem was raised in [Using GUI's with Docker](https://wiki.ros.org/es/docker/Tutorials/GUI#:~:text=%2D%2Dpulse.-,Using%20X%20server,-X%20server%20is)

  **For Linux:**
  ```bash
  # This command is required to run every time the PC is restarted
  xhost + 
  ```
  Make a X authentication file with proper permissions for the container to use.
  ```bash
  # If not working, try to run "sudo rm -rf /tmp/.docker.xauth" first
  cd ./src/beaverbot_dockerfiles/
  chmod +x ./install/xauth.sh && ./install/xauth.sh
  ```

  **For Windows:**
  ```powershell
  # Install VcXsrv or Xming X server
  # Start X server with these settings:
  # - Multiple windows
  # - Display number: 0
  # - Start no client
  # - Extra settings: Disable access control
  ```
  
* **Install Docker (optional)** 
 
  ```bash
  # For Linux
  chmod +x ./src/docker_installer.sh && ./src/install_docker.sh

  # For Windows: Download and install Docker Desktop from https://www.docker.com/products/docker-desktop
  ```

* **Launch the environment**
  > **Note**: For Windows users, to use USB devices (GPS, IMU, LiDAR), please follow the [USB Device Setup instructions](#usb-device-setup-for-windows) at the end of this file before launching the environment.

  ```bash
  cd .\src\beaverbot_dockerfiles
  docker compose -f [docker-compose-file] up -d
  ```

* **Open a container in interactive mode**

  ```bash
  cd .\src\beaverbot_dockerfiles
  docker compose -f [docker-compose-file] exec [name-of-container] bash
  ```

* **Stop containers**

  ```bash
  docker compose -f [docker-compose-file] down
  ```

* **Commit a container to a new image**
  ```bash
  # Do not do this if you're not familiar with Docker commit action. This changes your docker images.
  docker commit [container-id] [image-name:tag]
  ```

More other useful Docker's CLI can be found in [Docker CLI cheetsheet](https://docs.docker.com/get-started/docker_cheatsheet.pdf)

### Starting up the system with hardwares

-----

Start the environment

```bash
cd .\src\beaverbot_dockerfiles
docker compose -f [docker-compose-file] up -d
```

In terminal 1, run the below to enable sending command to robot and get the wheel encoder data

```bash
docker compose -f [docker-compose-file] exec robot_communication bash
roslaunch beaverbot_communication beaverbot_communication.launch
```

In terminal 2, run the below to start collect GPS and IMU sensor data

```bash
docker compose -f [docker-compose-file] exec beaverbot bash
roslaunch beaverbot_launch bringup.launch
```

### Scenario 1: Feedforward control

-----

In terminal 1 

```bash
cd ./src/beaverbot_dockerfiles
docker compose -f [docker-compose-file] up -d 
docker compose -f [docker-compose-file] exec robot_communication bash
roslaunch beaverbot_communication beaverbot_communication.launch
```

In terminal 2

```bash
docker compose -f [docker-compose-file] exec beaverbot bash
rosrun beaverbot_control feedforward
```

### Scenario 2: Feedback control with pure pursuit controller

-----

In terminal 1 

```bash
cd ./src/beaverbot_dockerfiles
docker compose -f [docker-compose-file] up -d 
docker compose -f [docker-compose-file] exec robot_communication bash
roslaunch beaverbot_communication beaverbot_communication.launch
```

In terminal 2

```bash
docker -f [docker-compose-file] compose exec beaverbot bash
roslaunch beaverbot_launch nav_pure_pursuit.launch
```

In terminal 3
```bash
docker -f [docker-compose-file] compose exec beaverbot -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && sleep 2 && roslaunch beaverbot_control beaverbot_control.launch"
```

## USB Device Setup for Windows

### Installation Steps

-----

1. Install USBIPD-WIN:
```powershell
winget install --interactive --exact dorssel.usbipd-win
```

2. Add USBIPD-WIN to system PATH:
```powershell
$env:Path += ";C:\Program Files\usbipd-win"; [Environment]::SetEnvironmentVariable("Path", $env:Path, [System.EnvironmentVariableTarget]::User)
```

3. **Important**: Close and reopen your terminal for PATH changes to take effect

### USB Device Management

-----

1. List available USB devices:
```powershell
# Shows device states:
# - Not shared: Device is available in Windows but not shared with WSL
# - Shared: Device is bound and ready to be attached to WSL  
# - Attached: Device is currently connected to WSL
usbipd list
```

2. **Run PowerShell as Administrator** and bind your devices:
```powershell
# This is a one-time setup for each device
# The binding persists across system restarts
usbipd bind -i [VID:PID-of-device]
```

3. Check your WSL distribution and load USB drivers for kernel:
```powershell
wsl --list
# Make sure 'docker-desktop' is listed in the output
wsl -d docker-desktop
modprobe ch341 
modprobe cp210x
modprobe ftdi_sio
exit
```

4. Attach devices to WSL:
```powershell
# This needs to be done after each:
# - System restart
# - Docker restart
# - Physical unplug/replug of device
usbipd attach --wsl docker-desktop -i [VID:PID-of-device]
```

5. Verify devices are accessible in container:
```bash
docker compose -f [docker-compose-file] up -d
docker compose exec beaverbot ls -l /dev
```

