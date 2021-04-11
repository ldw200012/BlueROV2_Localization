# bluerov-kalmanfilter
#### This repository is created for underwater robotics navigation for BlueROV2,
#### which reads the sensor value from DVL(Doppler Velocity Logs) and then derive the position of robot.
***

# I. How to install ROS Noetic
You can take a look at below ROS Wiki page to follow instructions to ROS Noetic installation

http://wiki.ros.org/noetic/Installation/Ubuntu

# II. How to create ROS Package
1. Get into your ROS workspace

       $ cd ~/catkin_ws/src
  
   - in case you have another name for your ROS workspace

         $ cd ~/<workspace_name>/src

2. Copy a ROS Package

       $ git clone https://github.com/ldw200012/bluerov_kalmanfilter.git


3. Run below commands to configure your ROS Package

       $ cd ~/catkin_ws
       $ catkin_make
       $ source devel/setup.bash
  
   - in case you have another name for your ROS workspace

         $ cd ~/<workspace_name>
         $ catkin_make
         $ source devel/setup.bash

