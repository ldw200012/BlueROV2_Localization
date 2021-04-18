# <div align=center>bluerov-kalmanfilter</div>
#### <div align="center">" This repository is created for underwater robotics navigation for BlueROV2, </div>
#### <div align="center"> which reads the sensor value from DVL(Doppler Velocity Logs) and derive the position of robot via kalman filter. "</div>

<div>
       <img src="https://bluerobotics.com/wp-content/uploads/2016/06/BlueROV2-4-lumen-1.png" width="50%">
       <img src="https://waterlinked.com/wp-content/uploads/2020/03/DSC04478_1600_web.jpg" width="40%">
</div><br>

<div align="center">We are subscribing <a href="https://waterlinked.github.io/docs/dvl/dvl-protocol/">DVL sensor values</a>, <a href="http://docs.ros.org/en/hydro/api/underwater_sensor_msgs/html/msg/DVL.html">underwater_sensor_msgs/DVL.msg</a><br><br>

then publishing the robot navigation value in a message type <a href="http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html">nav_msgs/Odometry.msg</a></div>
***

# I. Introduction to Navigation
https://www.nortekgroup.com/knowledge-center/wiki/new-to-subsea-navigation

# II. How to install ROS Noetic
You can take a look at below ROS Wiki page to follow instructions to ROS Noetic installation

http://wiki.ros.org/noetic/Installation/Ubuntu

# III. How to create ROS Workspace and Package
1. Create a ROS workspace

       $ mkdir -p ~/catkin_ws/src
       $ cd ~/catkin_ws/
       $ catkin_make
       $ source devel/setup.bash

2. Get into your ROS workspace

       $ cd ~/catkin_ws/src
       
3. Copy a ROS Package

       $ git clone https://github.com/ldw200012/bluerov_kalmanfilter.git

4. Run below commands to configure your ROS Package

       $ cd ~/catkin_ws
       $ catkin_make
       $ source devel/setup.bash (This command must be run on every shell you are using for ROS from now on)

# IV. How to use Kalman Filter
1. Theoretical understandings of Kalman Filter

    - https://www.kalmanfilter.net/default.aspx

2. Python implementation on Kalman Filter

    - https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html
