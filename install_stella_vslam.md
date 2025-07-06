# Stella_vslam_ros

## Install
### 01 dependencies
``` bash
sudo apt update -y
# basic dependencies
sudo apt install -y build-essential pkg-config cmake git wget curl unzip
# g2o dependencies
sudo apt install -y libatlas-base-dev libsuitesparse-dev
# OpenCV dependencies
sudo apt install -y libgtk-3-dev ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libtbb-dev
# eigen dependencies
sudo apt install -y gfortran
# backward-cpp dependencies (optional)
sudo apt install -y binutils-dev
# other dependencies
sudo apt install -y libyaml-cpp-dev libgflags-dev sqlite3 libsqlite3-dev
```

### 02 eigen
``` bash
cd /tmp
wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2
tar xf eigen-3.3.7.tar.bz2 && rm -rf eigen-3.3.7.tar.bz2
cd eigen-3.3.7
mkdir -p build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    ..
make -j4 && sudo make install
```

### 03 ros
``` bash
# use 1-2-1-1
##########################################################
wget http://fishros.com/install -O fishros && . fishros 
##########################################################

# Need to install opencv here. Fbow and g2o need opencv.
# If turn is not right, stella_vslam_ros will have Segmentation fault error.
pkg-config --modversion opencv4
# test on opencv4.5.4 by ros2 humble and opencv4.5.5+cvbridge from source
```

### 04 FBoW+g2o+backward-cpp+Pangolin
``` bash
cd /tmp
git clone https://github.com/stella-cv/FBoW.git
cd FBoW
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    ..
make -j4 && sudo make install

cd /tmp
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 20230223_git
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DG2O_USE_CHOLMOD=OFF \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_OPENMP=OFF \
    -DG2O_BUILD_APPS=OFF \
    -DG2O_BUILD_EXAMPLES=OFF \
    -DG2O_BUILD_LINKED_APPS=OFF \
    ..
make -j4 && sudo make install

cd /tmp
git clone https://github.com/bombela/backward-cpp.git
cd backward-cpp
git checkout 5ffb2c879ebdbea3bdb8477c671e32b1c984beaa
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    ..
make -j4 && sudo make install

cd /tmp
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout eab3d3449a33a042b1ee7225e1b8b593b1b21e3e
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PANGOLIN_DEPTHSENSE=OFF \
    -DBUILD_PANGOLIN_FFMPEG=OFF \
    -DBUILD_PANGOLIN_LIBDC1394=OFF \
    -DBUILD_PANGOLIN_LIBJPEG=OFF \
    -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
    -DBUILD_PANGOLIN_LIBPNG=OFF \
    -DBUILD_PANGOLIN_LIBTIFF=OFF \
    -DBUILD_PANGOLIN_LIBUVC=OFF \
    -DBUILD_PANGOLIN_LZ4=OFF \
    -DBUILD_PANGOLIN_OPENNI=OFF \
    -DBUILD_PANGOLIN_OPENNI2=OFF \
    -DBUILD_PANGOLIN_PLEORA=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF \
    -DBUILD_PANGOLIN_TELICAM=OFF \
    -DBUILD_PANGOLIN_UVC_MEDIAFOUNDATION=OFF \
    -DBUILD_PANGOLIN_V4L=OFF \
    -DBUILD_PANGOLIN_ZSTD=OFF \
    ..
make -j4 && sudo make install
```

### 05 stella_vslam+pangolin_viewer+stella_vslam_examples
``` bash
mkdir -p ~/lib
cd ~/lib
git clone --recursive https://github.com/stella-cv/stella_vslam.git
cd stella_vslam
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j4
sudo make install

cd ~/lib
git clone --recursive https://github.com/stella-cv/pangolin_viewer.git
mkdir -p pangolin_viewer/build
cd pangolin_viewer/build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j
sudo make install

cd ~/lib
git clone --recursive https://github.com/stella-cv/stella_vslam_examples.git
mkdir -p stella_vslam_examples/build
cd stella_vslam_examples/build
cmake \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DUSE_STACK_TRACE_LOGGER=ON \
    ..
make -j
```

### 06 fbow and example data
``` bash
cd ~/lib
wget https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow
# download from https://drive.google.com/drive/folders/1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4
# and unzip to ~/lib
```

### Run stella_vslam_examples
``` bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
cd ~/lib/stella_vslam_examples/build
./run_video_slam -v ~/lib/orb_vocab.fbow -c ~/lib/stella_vslam/example/aist/equirectangular.yaml -m ~/lib/aist_store_1/video.mp4
```


## stella_vslam ros2 install
``` bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive -b ros2 --depth 1 https://github.com/stella-cv/stella_vslam_ros.git
cd ~/ros2_ws
colcon build --symlink-install
```

## stella_vslam ros2 run
``` bash
source ~/ros2_ws/install/setup.bash
ros2 run stella_vslam_ros run_slam -v ~/lib/orb_vocab.fbow -c ~/lib/stella_vslam/example/aist/equirectangular.yaml --map-db-out ~/lib/map.msg  --ros-args -p publish_tf:=false
```