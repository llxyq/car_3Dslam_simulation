# Overview
常见的3D激光slam算法在仿真小车上的应用,包含A-LOAM，LIO-SAM，LEGO-LOAM

# Usage
A-LOAM使用Ceres Solver作为非线性求解器，所以安装A-LOAM之前需要安装Ceres Solver。
在网站http://ceres-solver.org/installation.html下载
使用LeGO-LOAM和LIO-SAM需安装[gtsam](https://github.com/borglab/gtsam/releases) 安装(Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)

在工作空间下使用catkin_make编译

1.加载仿真环境和小车：

roslaunch ackermann_vehicle_gazebo ackermann_vehicle_base.launch 

2.然后执行键盘控制节点：

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

3.LEGO-LOAM

roslaunch lego_loam run.launch 

4.LIO-SAM

roslaunch lio_sam run.launch 
