# robot_mower_2dnav
For a robot mower using rtabmap
Need to update...
# Path planner
This packege uses the waypoint generater from au-automow and heatmap  
au-automow https://github.com/Auburn-Automow/au-automow  
heatmap https://github.com/eybee/heatmap

# Way Point generator usage
clone mowing planner first
https://github.com/inuex3/mowing_planner.
After catkin_make,
run  
roslaunch robot_mower_2dnav planning_test.launch
rosrun robot_mower_2dnav heatmap_client  
and  
rosrun robot_mower_2dnav path_planner.py  
Draw a geometrycal figure with publish_point
 
