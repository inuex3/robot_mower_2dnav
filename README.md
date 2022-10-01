# robot_mower_2dnav
For a robot mower using rtabmap

YouTube Video

[![autonomous navigation for a robotic mower](https://user-images.githubusercontent.com/40222376/189482705-b24ad36b-9d6c-43f3-aeac-aa685ccd58ce.jpg)](https://www.youtube.com/watch?v=h_7y-r1qzcE)



# Installation 

```
sudo apt-get install ros-noetic-rtabmap-ros ros-noetic-teb-local-planner
```
If you want to use GPU, you need to build it from source.


Clone mowing planner and modified darknet_ros   
```
cd ~/catkin_ws/src
git clone https://github.com/inuex3/robot_mower_2dnav.git
git clone --recursive https://github.com/inuex3/darknet_ros.git
git clone https://github.com/inuex3/mowing_planner.git
cd ~/catkin_ws
catkin_make
```

# Localization from rosbag
Download rosbag from here.
https://drive.google.com/file/d/1l0IrpAPV1WoW48CIPvnGWnefiqSngt_b/view?usp=sharing

Download weight for yolov3.
https://drive.google.com/file/d/13GLpu-C4q3wrMYeovnqS2d8iY-RM0yv8/view?usp=sharing

Put the downloaded weight in darknet_ros/darknet_ros/yolo_network_config/weights

```
roslaunch robot_mower_2dnav sim_slam.launch
rosbag play forwardbackward.bag --clock
```

# To do

Create the simulation environment with AirSim.

# Path planner
This packege uses the waypoint generater from au-automow and heatmap  
au-automow https://github.com/Auburn-Automow/au-automow  
heatmap https://github.com/eybee/heatmap

# Way Point generator usage
run  
rosrun robot_mower_2dnav heatmap_client  
and  
rosrun robot_mower_2dnav path_planner.py  
Draw a geometrycal figure with publish_point
 
