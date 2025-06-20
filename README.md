# 360-Degree Panoramic Camera VSLAM System

[中文说明](https://github.com/Longxiaoze/360Vslam/blob/main/readme-cn.md)

## Project Overview
This project aims to develop a visual simultaneous localization and mapping (VSLAM) system based on a 360-degree panoramic camera. The system is built on [stella_slam](https://github.com/stella-cv/stella_vslam) and uses the Insta360 X4 panoramic camera. VSLAM is a technique that uses visual information to enable a robot or device to localize itself and build a map in an unknown environment. It is widely applied in autonomous driving, robot navigation, augmented reality, and other fields.

## Features
- **Panoramic Vision Support**: Captures full-environment information using a 360-degree panoramic camera.
- **Real-Time Localization**: Achieves high-precision real-time localization via visual feature matching and optimization algorithms.
- **Map Construction**: Generates a sparse 3D map of the environment.
- **Map Reuse**: Supports relocalization using a prior map built with either the panoramic camera or another camera type.
- **Modular Design**: Employs a modular architecture for easy extension and maintenance.

## Environment Dependencies
- **Programming Language**: C++
- **Hardware Requirements**:  
  - Insta360 X4 360-degree panoramic camera

---

# 1. Using the Panoramic Camera

## 1.1) Obtain the SDK
- https://www.insta360.com/cn/sdk/record  
- https://github.com/Insta360Develop/  

## 1.2) Connect the Camera  
**Insta360 X4**  
Connect the device via USB and select “Android” mode on the camera’s screen.

On Linux, ensure your distribution has libusb installed. You can install via yum or apt-get:
```bash
sudo apt-get install libusb-dev
sudo apt-get install libudev-dev
````

Or build from source:

```bash
wget http://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.9/libusb-1.0.9.tar.bz2
tar xjf libusb-1.0.9.tar.bz2
cd libusb-1.0.9
./configure
make
sudo make install
```

After installing the driver, verify camera detection with:

```bash
lsusb
```

If you see any USB device with vendor ID `0x2e1a`, your driver installation was successful. For example:

```bash
Bus 001 Device 012: ID 2e1a:0002 SEM USB Keyboard
```

## 1.3) Install the SDK

Unzip the downloaded SDK file (e.g., `LinuxSDK20241128.zip`), then:

```bash
sudo dpkg -i libMediaSDK-dev_2.0-6_amd64_ubuntu18.04.deb
```

> **Note:** On Linux, demo programs must be run with `sudo`. For example:
>
> ```bash
> sudo ./CameraSDKDemo   # for Ubuntu
> ```

---

# 2. Running stella\_vslam

Follow the installation guides for [stella\_vslam](https://stella-cv.readthedocs.io/en/latest/installation.html) and [stella\_vslam\_ros2](https://stella-cv.readthedocs.io/en/latest/ros2_package.html#installation):

```bash
sudo apt install ros-humble-image-publisher

# Publish prerecorded video as a ROS2 image stream
ros2 run image_publisher image_publisher_node /home/longxiaoze/Downloads/aist_living_lab_1/video.mp4 \
  --ros-args --remap /image_raw:=/camera/image_raw

# Source your workspace
source ~/ros2_ws/install/setup.bash

# Repeat if needed
ros2 run image_publisher image_publisher_node /home/longxiaoze/Downloads/aist_living_lab_1/video.mp4 \
  --ros-args --remap /image_raw:=/camera/image_raw
```

---

# 3. Connecting the Panoramic Camera and Running with stella\_vslam

## 3.1) Install the Insta360 ROS Package

Follow [https://github.com/ai4ce/insta360\_ros\_driver](https://github.com/ai4ce/insta360_ros_driver)

```bash
mkdir -p ~/360_ws/src
cd ~/360_ws/src

# Clone the ROS driver for Humble
git clone -b humble https://github.com/Longxiaoze/insta360_ros_driver.git

# Copy SDK headers and library into the driver package
cp -r path/to/LinuxSDK20241128/CameraSDK-20241120_183228--1.1.0-Linux/include/camera/ \
      ~/360_ws/src/insta360_ros_driver/include/
cp -r path/to/LinuxSDK20241128/CameraSDK-20241120_183228--1.1.0-Linux/include/stream/ \
      ~/360_ws/src/insta360_ros_driver/include/
cp path/to/LinuxSDK20241128/CameraSDK-20241120_183228--1.1.0-Linux/lib/libCameraSDK.so \
      ~/360_ws/src/insta360_ros_driver/lib/

# Build the workspace
cd ~/360_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Install Python dependencies
sudo apt install python3-pip
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 \
  --index-url https://download.pytorch.org/whl/cu118

sudo apt update
sudo apt install ros-humble-tf2-tools
```

## 3.2) Calibrate the Insta360 X4 Fisheyes with ROS2

### 3.2.1 Calibration

Run the camera driver:

```bash
source ~/360_ws/install/setup.bash
ls /dev/insta
sudo chmod 777 /dev/insta
ros2 run insta360_ros_driver insta360_ros_driver
```

Activate image decoding:

```bash
source ~/360_ws/install/setup.bash
ros2 run insta360_ros_driver decoder
```

Run the equirectangular node in calibration mode:

```bash
# Option A: direct script
python3 ~/360_ws/src/insta360_ros_driver/scripts/calibrate.py

# Or, if the executable is set up:
ros2 run insta360_ros_driver equirectangular.py --calibrate
```

Open RViz2 to inspect the calibration:

```bash
rviz2
```

## 3.3) Publish Insta360 X4 360° Images via ROS2

```bash
ls /dev/insta
sudo chmod 777 /dev/insta

source ~/360_ws/install/setup.bash
cd ~/360_ws/src/insta360_ros_driver/
./setup.sh
ros2 launch insta360_ros_driver bringup.launch.xml
```

Then, in another terminal:

```bash
rviz2
```

Finally, run SLAM with the published 360° stream:

```bash
source ~/ros2_ws/install/setup.bash
cd ~/360_ws/src/insta360_ros_driver/config
wget https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow

# Execute SLAM
ros2 run stella_vslam_ros run_slam \
  -v ./config/orb_vocab.fbow \
  -c ./config/insta360X4_equirectangular.yaml \
  --map-db-out map.msg \
  --ros-args -p publish_tf:=false
```

