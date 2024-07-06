# Livox-related-SLAM-application
A summary of Livox-related SLAM-application: compiling and execution 
+ Single-LiDAR: [LiLi-OM](https://github.com/KIT-ISAS/lili-om), [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), [Loam-Livox](https://github.com/hku-mars/loam_livox), [FAST-LIO 1&2](https://github.com/hku-mars/FAST_LIO), [Faster-LIO](https://github.com/gaoxiang12/faster-lio), [LIO-Livox](https://github.com/Livox-SDK/LIO-Livox), [Livox-mapping](https://github.com/PJLab-ADG/Livox-Mapping), [DLO](https://github.com/vectr-ucla/direct_lidar_odometry), [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry),[Point-LIO](https://github.com/hku-mars/Point-LIO), [BALM](https://github.com/hku-mars/BALM), [VoxelMap](https://github.com/hku-mars/VoxelMap), [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public), [PV-LIO](https://github.com/HViktorTsoi/PV-LIO), and [ImMesh](https://github.com/hku-mars/ImMesh).

+ Multi-LiDARs: [FAST-LIO-MULTI](https://github.com/engcang/FAST_LIO_MULTI)



## Videos: 
### Single-LiDAR
+ [`video1`](https://youtu.be/3d4WtK6S4Ms): LIO-SAM vs FAST-LIO2
+ [`video2`](https://youtu.be/NmT0o268OLM): FAST-LIO2 vs Livox-mapping vs LOAM-Livox using [`Livox Mid-70 LiDAR`](https://www.livoxtech.com/mid-70), real-world
+ [`video3`](https://youtu.be/_RgtOdK53z4): FAST-LIO2 in the building with narrow stairs using [`Ouster OS0-128`](https://ouster.com/products/scanning-lidar/os0-sensor/), real-world
+ [`video4`](https://youtu.be/emiSJMcA8yM): FAST-LIO2 in the narrow tunnels using [`Ouster OS0-128`](https://ouster.com/products/scanning-lidar/os0-sensor/) on the UAV (drone)
+ [`video5`](https://youtu.be/kr9Z2e8I7YQ): Faster-LIO vs FAST-LIO in the building with narrow stairs using [`Ouster OS0-128`](https://ouster.com/products/scanning-lidar/os0-sensor/), real-world
+ [`video6`](https://youtu.be/7m2XBfcgJPM): VoxelMap in the building using [`Intel Realsense L515`](https://www.intelrealsense.com/lidar-camera-l515/), real-world
+ [`video7`](https://youtu.be/NYaT-bxGjvY): R3LIVE in the building and around the building using [`Livox Mid-70 LiDAR`](https://www.livoxtech.com/mid-70), [`FLIR Blackfly S`](https://www.flirkorea.com/products/blackfly-s-usb3/), [`Pixhawk4 mini`](http://www.holybro.com/product/pixhawk4-mini/), real-world
+ [`video8`](https://youtu.be/3QTjUONviYo): FAST-LIO2 vs Ada-LIO vs Point-LIO vs KISS-ICP in the building with narrow stairs, real-world
+ [`video9`](https://youtu.be/vxFBvXTuFoU): FAST-LIO2 vs Ada-LIO in Gazebo challenging environments
+ [`video10`](https://youtu.be/0udb2fRG6HY): DLO vs FAST-LIO2
+ [`video11`](https://youtu.be/r9wwldPYKUw): DLO vs DLIO vs Ada-LIO vs FAST-LIO2
+ [`video12`](https://youtu.be/NHPvLbztawY): Ada-LIO vs PV-LIO vs SLAMesh
+ [`video13`](https://youtu.be/0sVkY66pOpI): Ada-LIO vs PV-LIO
+ [`video14`](https://youtu.be/rOOKY9HVHzI): Ada-LIO vs ImMesh
### Multi-LiDARs
+ [`video15`](https://youtu.be/YQmjKMoBPNU): FAST-LIO-MULTI bundle update vs asynchronous update
---



## Dependencies
+ Common packages
~~~bash
sudo apt-get install -y ros-noetic-navigation ros-noetic-robot-localization ros-noetic-robot-state-publisher
~~~

+ [GTSAM](https://github.com/borglab/gtsam/releases) for `LVI-SAM` and `LIO-SAM`
~~~bash
wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
unzip gtsam.zip
cd gtsam-4.0.2/
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install -j8
~~~

+ [Ceres solver](http://ceres-solver.org) for `LVI-SAM` and `SLAMesh`
~~~bash
sudo apt-get install -y cmake libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev
wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz #LVI-SAM
wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz #SLAMesh
tar zxf ceres-solver-1.14.0.tar.gz #LVI-SAM
tar zxf ceres-solver-2.1.0.tar.gz #SLAMesh
mkdir ceres-bin
mkdir solver && cd ceres-bin
cmake ../ceres-solver-1.14.0 -DEXPORT_BUILD_DIR=ON -DCMAKE_INSTALL_PREFIX="../solver"  #good for build without being root privileged and at wanted directory
make -j8 # 8 : number of cores
make test
make install
~~~

+ `glog`, `g++-9` and `gcc-9` for `Faster-LIO`
~~~bash
sudo apt-get install libgoogle-glog-dev
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-9 g++-9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-9
~~~


## Compiling


### ● LiLi-OM
~~~shell
cd ~/catkin_ws/src
git clone https://github.com/KIT-ISAS/lili-om
cd ..
catkin build livox_ros_driver
catkin build lili_om
catkin build lili_om_rot
~~~

### ● LIO-SAM
~~~shell
cd ~/your_workspace/src
git clone https://github.com/TixiaoShan/LIO-SAM.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
~~~
#### ● You can test the livox data with Pointcloud2 format input.

##

### ● Loam-Livox
~~~shell
cd ~/your_workspace/src
git clone https://github.com/hku-mars/loam_livox.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
~~~
#### ● You can test the livox horizon or hap with [livox-horizon-loam](https://github.com/Livox-SDK/livox_horizon_loam).


### ● FAST-LIO 1 & 2
~~~shell
cd ~/your_workspace/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
~~~


### ● Faster-LIO
~~~shell
cd ~/your_workspace/src
git clone https://github.com/gaoxiang12/faster-lio.git

cd faster-lio/thirdparty
tar -xvf tbb2018_20170726oss_lin.tgz

cd ~/your_workspace
catkin build -DCUSTOM_TBB_DIR=$(pwd)/src/faster-lio/thirdparty/tbb2018_20170726oss -DCMAKE_BUILD_TYPE=Release
~~~
#### ● You need modify the launch file when use the horizon.launch

### ● Lio-livox
~~~shell
cd ~/your_workspace/src
git clone https://github.com/Livox-SDK/LIO-Livox.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
~~~

### ● livox-mapping
~~~shell
cd ~/your_workspace/src
git clone https://github.com/PJLab-ADG/Livox-Mapping.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
~~~

### ● BALM-old
~~~shell
cd ~/your_workspace/src
git clone https://github.com/hku-mars/BALM.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
~~~

##

### ● VoxelMap
```bash
cd ~/your_workspace/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release

git clone https://github.com/hku-mars/VoxelMap.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
```

### ● VoxelMap++
```bash
cd ~/your_workspace/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release

git clone https://github.com/uestc-icsp/VoxelMapPlus_Public.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
```

### ● PV-LIO
```shell
cd ~/your_workspace/src
git clone https://github.com/Livox-SDK/livox_ros_driver
git clone https://github.com/HViktorTsoi/PV-LIO
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
```

##



### ● DLO
```shell
sudo apt install libomp-dev libpcl-dev libeigen3-dev 
cd ~/your_workspace/src
git clone https://github.com/vectr-ucla/direct_lidar_odometry.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
```


### ● DLIO
```shell
sudo apt install libomp-dev libpcl-dev libeigen3-dev 
cd ~/your_workspace/src
git clone https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
```

##

### ● Point-LIO
~~~shell
cd ~/your_workspace/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release

cd ~/your_workspace/src
git clone https://github.com/hku-mars/Point-LIO.git
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
~~~


### ● FAST-LIO-MULTI
```shell
cd ~/your_workspace/src
git clone https://github.com/Livox-SDK/livox_ros_driver
git clone https://github.com/engcang/FAST_LIO_MULTI
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
```


### ● ImMesh
```shell
sudo apt-get install ros-noetic-cv-bridge ros-noetic-tf ros-noetic-message-filters ros-noetic-image-transport*
sudo apt-get install -y libcgal-dev pcl-tools libgl-dev libglm-dev libglfw3-dev libglew-dev libglw1-mesa-dev libxkbcommon-x11-dev
sudo apt-get install -y libxinerama-dev
cd ~/your_workspace/src
git clone https://github.com/Livox-SDK/livox_ros_driver
catkin_make
cd ~/your_workspace/src
git clone https://github.com/hku-mars/ImMesh
cd ..
catkin_make
```


## How to run
#### ● check each of config files and launch files in the folders of this repo

