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

## 1) 申请sdk
https://www.insta360.com/cn/sdk/record

https://github.com/Insta360Develop/


## 2） 连接相机
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

## 3）安装SDK
解压申请之后的sdk文件LinuxSDK20241128.zip
``` bash
sudo dpkg -i libMediaSDK-dev_2.0-6_amd64_ubuntu18.04.deb 

```
注意：在Linux上，演示程序必须通过“ sudo ”运行，例如
``` bash
sudo ./CameraSDKDemo //for ubuntu
```

## 3）安装驱动程序

# 2. stella_vslam 的运行

# 3. 连接全景相机，使用stella_vslam运行