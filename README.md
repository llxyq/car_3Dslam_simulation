# Overview

常见的3D激光slam算法在仿真小车上的应用,包含A-LOAM，LIO-SAM，LEGO-LOAM，FAST-LIO

## 1. Prerequisites

小车模型来自开源项目：[ackermann_vehicle](https://github.com/hdh7485/ackermann_vehicle)。

A-LOAM开源地址：https://github.com/HKUST-Aerial-Robotics/A-LOAM

LEGO-LOAM开源地址：https://github.com/RobustFieldAutonomyLab/LeGO-LOAM

LIO-SAM开源地址：https://github.com/TixiaoShan/LIO-SAM

FAST-LIO开源地址：https://github.com/hku-mars/FAST_LIO

**运行环境**：

Ubuntu 64-bit 20.04. ROS Noetic

**前置依赖：**

1. velodyne激光模拟功能包：

   ```
   sudo apt-get install ros-noetic-velodyne-*
   ```

2. 阿克曼消息功能包：

   ```
   sudo apt install ros-noetic-ackermann-msgs
   ```

3. 键盘控制节点功能包：

   ```
   sudo apt install ros-noetic-teleop-twist-keyboard
   ```

4. 其他依赖

   ```
   sudo apt-get install ros-noetic-fake-localization
   ```

   ```
   sudo apt-get install ros-noetic-robot-localization
   ```

### 1.2 Ceres Solver非线性求解器安装

A-LOAM需要Ceres Solver非线性求解器，在网站 http://ceres-solver.org/installation.html 下载，推荐下载稳定版`ceres-solver-2.2.0`。

前置依赖：

```
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev
```

编译安装：

```
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.2.0
make -j3
make test
sudo make install               
```

### 1.3 gtsam安装

LeGO-LOAM和LIO-SAM需gtsam，手动下载[gtsam-4.0.0-alpha2](https://github.com/borglab/gtsam/tree/4.0.0-alpha2) 的压缩包，并解压。

```
cd ~/gtsam-4.0.0-alpha2/
mkdir build && cd build
cmake ..
sudo make install
```



## 2. Usage

### 2.1 Build

```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

编译可能出现以下报错：

```
/usr/include/pcl-1.10/pcl/filters/voxel_grid.h:340:21: error: ‘Index’ is not a member of ‘Eigen’
  340 |         for (Eigen::Index ni = 0; ni < relative_coordinates.cols (); ni++)
/usr/include/pcl-1.10/pcl/filters/voxel_grid.h:669:35: error: ‘ni’ was not declared in this scope
  669 |         for (Eigen::Index ni = 0; ni < relative_coordinates.cols (); ni++)

```

解决办法：

根据提示找到文件`/usr/include/pcl-1.10/pcl/filters/voxel_grid.h`

```
cd /usr/include/pcl-1.10/pcl/filters/
sudo gedit voxel_grid.h
```

将340和669行的

```
for (Eigen::Index ni = 0; ni < relative_coordinates.cols (); ni++)
```

改为

```
for (int ni = 0; ni < relative_coordinates.cols (); ni++)
```

除此之外**[lio_sam_imuPreintegration-2]** 这个模块启动可能会报错

使用下面命令即可

```
cd /usr/local/lib/
sudo cp libmetis.so /opt/ros/noetic/lib/
```



### 2.2 Running

加载仿真世界和车模：

```
roslaunch ackermann_vehicle_gazebo ackermann_vehicle_base.launch
```

LEGO-LOAM建图：

```
roslaunch lego_loam run.launch
```

LIO-SAM建图：

```
roslaunch lio_sam run.launch
```

A-LOAM建图：

```
roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
```

键盘控制节点：

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

