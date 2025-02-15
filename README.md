# car_3Dslam_simulation
常见的3D激光slam算法在仿真小车上的应用,包含A-LOAM，LIO-SAM，LEGO-LOAM
1.加载仿真环境和小车：

roslaunch ackermann_vehicle_gazebo ackermann_vehicle_base.launch 
2.然后执行键盘控制节点：
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
3.LEGO-LOAM
 roslaunch lego_loam run.launch 
4.LIO-SAM
roslaunch lio_sam run.launch 
