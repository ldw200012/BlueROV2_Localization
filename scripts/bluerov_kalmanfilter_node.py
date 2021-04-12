#!/usr/bin/env python

import rospy
from bluerov_kalmanfilter.msg import DVL
from nav_msgs.msg import Odometry
from filterpy.kalman import KalmanFilter

def callback(data):
    
    # data: DVL.msg
    # bluerov_odom: Odometry.msg
    
    # Kalman Filter here 
    
    
    
    # return bluerov_odom
    
    pub.publish(bluerov_odom)

def navigation():
    
    rospy.init_node('bluerov_kalmanfilter_node')
    rate = rospy.Rate(10) # 10hz
    pub = rospy.Publisher("bluerov_odometry", String, queue_size=10)
    
    while not rospy.is_shutdown():
        sub = rospy.Subscriber("bluerov_dvl", String, callback)
        rate.sleep()

        
if __name__ == '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
