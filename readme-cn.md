# 360度全景相机的VSLAM系统

## 项目简介
本项目旨在开发一个基于360度全景相机的视觉同时定位与建图（VSLAM）系统。本系统基于[stella_slam](https://github.com/stella-cv/stella_vslam）构建，使用insta360 X4全景相机。
VSLAM是一种利用视觉信息实现机器人或设备在未知环境中定位和构建地图的技术，广泛应用于自动驾驶、机器人导航和增强现实等领域。

## 功能特性
- **全景视觉支持**：利用360度全景相机捕获全方位环境信息。
- **实时定位**：通过视觉特征匹配和优化算法实现高精度的实时定位。
- **地图构建**：生成稀疏的三维环境地图。
- **地图复用**：支持使用全景相机或者其他相机的先验地图上进行重定位。
- **模块化设计**：系统采用模块化架构，便于扩展和维护。

## 环境依赖
- **编程语言**：C++

- **硬件要求**：
    - 360度全景相机 insta360 X4


# 1. 全景相机的使用

## 1.1) 申请sdk
https://www.insta360.com/cn/sdk/record

https://github.com/Insta360Develop/


## 1.2） 连接相机
X4
通过 USB 连接设备并在相机屏幕上选择Android 模式。

在 Linux 上，请确保您的发行版已安装 libusb。

您可以通过 yum 或 apt-get 安装
``` bash
sudo apt-get install libusb-dev
sudo apt-get install libudev-dev
```
或从源代码构建
``` bash
wget http://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.9/libusb-1.0.9.tar.bz2
tar xjf libusb-1.0.9.tar.bz2
cd libusb-1.0.9
./configure 
make
sudo make install
```
安装驱动后，通过命令检查是否检测到相机lsusb，如果发现任何供应商 ID 为 0x2e1a 的 USB 设备，恭喜您，您的驱动已成功安装。
如：
``` bash
Bus 001 Device 012: ID 2e1a:0002 SEM USB Keyboard
```

## 1.3）安装SDK
解压申请之后的sdk文件LinuxSDK20241128.zip
``` bash
sudo dpkg -i libMediaSDK-dev_2.0-6_amd64_ubuntu18.04.deb 

```
注意：在Linux上，演示程序必须通过“ sudo ”运行，例如
``` bash
sudo ./CameraSDKDemo //for ubuntu
```


# 2. stella_vslam 的运行
follow [stella_vslam](https://stella-cv.readthedocs.io/en/latest/installation.html) and [stella_vslam_ros2](https://stella-cv.readthedocs.io/en/latest/ros2_package.html#installation) 

``` bash
sudo apt install ros-humble-image-publisher
ros2 run image_publisher image_publisher_node /home/longxiaoze/Downloads/aist_living_lab_1/video.mp4  --ros-args --remap /image_raw:=/camera/image_raw
source ~/ros2_ws/install/setup.bash
ros2 run image_publisher image_publisher_node /home/longxiaoze/Downloads/aist_living_lab_1/video.mp4  --ros-args --remap /image_raw:=/camera/image_raw
ros2 run image_publisher image_publisher_node /home/longxiaoze/Downloads/aist_living_lab_1/video.mp4  --ros-args --remap /image_raw:=/camera/image_raw
```


# 3. 连接全景相机，使用stella_vslam运行
## 3.1）安装insta360 ros package
follow https://github.com/ai4ce/insta360_ros_driver

``` bash
mkdir -p ~/360_ws/src
cd ~/360_ws/src
# git clone -b humble https://github.com/ai4ce/insta360_ros_driver
git clone -b humble https://github.com/Longxiaoze/insta360_ros_driver.git

cp -r path/to/LinuxSDK20241128/CameraSDK-20241120_183228--1.1.0-Linux/include/camera/ ~/360_ws/src/insta360_ros_driver/include/
cp -r path/to/LinuxSDK20241128/CameraSDK-20241120_183228--1.1.0-Linux/include/stream/ ~/360_ws/src/insta360_ros_driver/include/
cp path/to/LinuxSDK20241128/CameraSDK-20241120_183228--1.1.0-Linux/lib/libCameraSDK.so ~/360_ws/src/insta360_ros_driver/lib/

cd ~/360_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

sudo apt install python3-pip
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 --index-url https://download.pytorch.org/whl/cu118
sudo apt update
sudo apt install ros-humble-tf2-tools
```

## 3.2）Calib insta360 X4 fisheyes by ros2
### 3.2.1 calib
Run the camera driver
``` bash
source ~/360_ws/install/setup.bash
ls /dev/insta
sudo chmod 777 /dev/insta
ros2 run insta360_ros_driver insta360_ros_driver
```

Activate image decoding
``` bash
source ~/360_ws/install/setup.bash
ros2 run insta360_ros_driver decoder
```

Run the equirectangular node in calibration mode
``` bash
# ros2 run insta360_ros_driver equirectangular.py --calibrate
python3 ~/360_ws/src/insta360_ros_driver/scripts/calibrate.py
```

open rviz2 to check calib
``` bash
rviz2
```

## 3.3）Publish insta360 X4 360 images by ros2
``` bash
ls /dev/insta
sudo chmod 777 /dev/insta

source ~/360_ws/install/setup.bash
cd ~/360_ws/src/insta360_ros_driver/
./setup.sh
ros2 launch insta360_ros_driver bringup.launch.xml
```

``` bash
rviz2
```


``` bash
source ~/ros2_ws/install/setup.bash
cd ~/360_ws/src/insta360_ros_driver/config
wget https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow
cd ~/360_ws/src/insta360_ros_driver
ros2 run stella_vslam_ros run_slam     -v  ./config/orb_vocab.fbow  -c ./config/insta360X4_equirectangular.yaml  --map-db-out  map.msg     --ros-args -p publish_tf:=false
```



