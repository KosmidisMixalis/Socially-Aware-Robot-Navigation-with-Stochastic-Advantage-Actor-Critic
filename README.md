## Socially-Aware Navigation via Stochastic Advantage Actor-Critic for Continuous Action Control in POMDPs

## Overview

roslaunch ridgeback_navigation gmapping_demo.launch 
roslaunch ridgeback_viz view_robot.launch config:=gmapping 
rosrun map_server map_saver -f mymap This will create a mymap.yaml and mymap.pgm file in your current directory. 
roslaunch ridgeback_navigation amcl_demo.launch map_file:=/path/to/my/map.yaml 
roslaunch ridgeback_viz view_robot.launch config:=localization




roslaunch ridgeback_gazebo ridgeback_world.launch
roslaunch ridgeback_navigation amcl_demo.launch map_file:=/home/alien/Desktop/catkin_ws/src/Social_Nav_Src/test/mymap.yaml
roslaunch ridgeback_viz view_robot.launch config:=localization


## Authors

- **Michail Kosmidis**
- **Ioannis Kansizoglou**
- **Antonios Gasteratos**

The authors are affiliated with the **Laboratory of Robotics & Automation (LRA)** and the **Department of Production and Management Engineering**, **Democritus University of Thrace, Xanthi 67100, Greece**.

🔗 [Lab Website](https://robotics.pme.duth.gr)


