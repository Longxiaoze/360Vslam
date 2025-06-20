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
