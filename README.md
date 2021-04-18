# <div align=center>bluerov-kalmanfilter</div>
#### <div align="center">" This repository is created for underwater robotics navigation for BlueROV2, </div>
#### <div align="center"> which reads the sensor value from DVL(Doppler Velocity Logs) and derive the position of robot via kalman filter. "</div>

<div>
       <img src="https://bluerobotics.com/wp-content/uploads/2016/06/BlueROV2-4-lumen-1.png" width="50%">
       <img src="https://waterlinked.com/wp-content/uploads/2020/03/DSC04478_1600_web.jpg" width="40%">
</div>

<div align="center">We are subscribing DVL sensor value, which has a message type <a href="http://docs.ros.org/en/hydro/api/underwater_sensor_msgs/html/msg/DVL.html">underwater_sensor_msgs/DVL.msg</a><br><br>

then publishing the robot navigation value in a message type <a href="http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html">nav_msgs/Odometry.msg</a></div>
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

# III. How to use Kalman Filter in Python
1. Theoretical understandings of Kalman Filter

    - https://www.kalmanfilter.net/default.aspx

2. Python implementation on Kalman Filter

    - https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html
