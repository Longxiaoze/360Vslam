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
follow https://stella-cv.readthedocs.io/en/latest/installation.html

# 3. 连接全景相机，使用stella_vslam运行
## 3.1）安装insta360 ros package
follow https://github.com/ai4ce/insta360_ros_driver

``` bash
mkdir -p ~/360_ws/src
cd ~/360_ws/src
git clone -b humble https://github.com/ai4ce/insta360_ros_driver

cp -r path/to/LinuxSDK20241128/CameraSDK-20241120_183228--1.1.0-Linux/include/camera/ ~/360_ws/src/insta360_ros_driver/include/
cp -r path/to/LinuxSDK20241128/CameraSDK-20241120_183228--1.1.0-Linux/include/stream/ ~/360_ws/src/insta360_ros_driver/include/
cp path/to/LinuxSDK20241128/CameraSDK-20241120_183228--1.1.0-Linux/lib/libCameraSDK.so ~/360_ws/src/insta360_ros_driver/lib/

cd ~/360_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

code changing for insta X4: follow [issue](https://github.com/ai4ce/insta360_ros_driver/issues/13#issuecomment-2727005037)

(1) line 100 in ~/360_ws/src/insta360_ros_driver/src/main.cpp
``` bash
 -        img_msg->encoding = "rgb8";
 +        img_msg->encoding = "bgr8";
```
(2) void OnVideoData() in ~/360_ws/src/insta360_ros_driver/src/main.cpp
``` c++
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index = 0) override {
        if (stream_index == 0) {
            pkt->data = const_cast<uint8_t*>(data);
            pkt->size = size;
        }
    
        if (avcodec_send_packet(codecCtx, pkt) == 0) {
            while (avcodec_receive_frame(codecCtx, avFrame) == 0) {
                int width  = avFrame->width;
                int height = avFrame->height;
    
                // Calculate required YUV buffer size (I420 format)
                const int yuv_buffer_size = av_image_get_buffer_size(AV_PIX_FMT_YUV420P, width, height, 1);
                std::vector<uint8_t> yuv_buffer(yuv_buffer_size);
    
                // Efficiently copy YUV data to a continuous buffer
                av_image_copy_to_buffer(yuv_buffer.data(), yuv_buffer_size,
                                        avFrame->data, avFrame->linesize,
                                        AV_PIX_FMT_YUV420P, width, height, 1);
    
                // Directly construct cv::Mat from the buffer (no additional copies)
                cv::Mat yuv(height + height / 2, width, CV_8UC1, yuv_buffer.data());
    
                // Efficient color conversion
                cv::Mat rgb;
                cv::cvtColor(yuv, rgb, cv::COLOR_YUV2BGR_I420);
    
                // Efficient image slicing (no copies, only references)
                int midPoint = width / 2;
                cv::Mat frontImage = rgb(cv::Rect(midPoint, 0, midPoint, height));
                cv::Mat backImage  = rgb(cv::Rect(0, 0, midPoint, height));
    
                // Uncomment if rotating is necessary:
                // frontImage = rotateImage(frontImage, 90);
                // backImage = rotateImage(backImage, -90);
    
                cv::Mat dualFisheyeImage;
                cv::hconcat(frontImage, backImage, dualFisheyeImage);
    
                auto dualFisheyeMsg = matToImgMsg(dualFisheyeImage, "dual_fisheye_frame");
                dual_fisheye_pub_->publish(*dualFisheyeMsg);
            }
        }
    }
```