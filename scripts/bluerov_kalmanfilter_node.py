#!/usr/bin/env python

import rospy
from bluerov_kalmanfilter.msg import DVL
from nav_msgs.msg import Odometry
from filterpy.kalman import KalmanFilter
from std_msgs.msg import String

saved_child_frame_id = "None"

def callback(data):
    
    # data: DVL.msg ( we work on #BOTTOM-TRACK, INSTRUMENT-REFERENCED VELOCITY DAT)
    # bluerov_odom: Odometry.msg
    
    DVL = data
    
    pos_x = 0
    pos_y = 0
    pos_z = 0
    
    vel_x = DVL.wi_x_axis
    vel_y = DVL.wi_y_axis
    vel_z = DVL.wi_z_axis
    
    wi_error = DVL.wi_error
    wi_status = DVL.wi_status
    
    # Kalman Filter here    
    
    
    
    # Output Odometry message
    bluerov_odom = Odometry()
    
    bluerov_odom.header.seq = DVL.header.seq
    bluerov_odom.header.stamp = DVL.header.stamp
    bluerov_odom.header.frame_id = DVL.header.frame_id
    
    bluerov_odom.header.child_frame_id = saved_child_frame_id
    saved_child_frame_id = DVL.header.frame_id
    
    bluerov_odom.pose.position.x = 
    bluerov_odom.pose.position.y = 
    bluerov_odom.pose.position.z = 
    
#     bluerov_odom.pose.orientation.x = 
#     bluerov_odom.pose.orientation.y = 
#     bluerov_odom.pose.orientation.z = 
#     bluerov_odom.pose.orientation.w = 
    
#     bluerov_odom.pose.covariance = 
    
    bluerov_odom.twist.linear.x = DVL.wi_x_axis
    bluerov_odom.twist.linear.y = DVL.wi_y_axis
    bluerov_odom.twist.linear.z = DVL.wi_z_axis
    
#     bluerov_odom.twist.angular.x = 
#     bluerov_odom.twist.angular.y = 
#     bluerov_odom.twist.angular.z = 
    
#     bluerov_odom.twist.covariance = 
    
    pub = rospy.Publisher("bluerov_odometry", Odometry, queue_size=10)
    pub.publish(bluerov_odom)

def navigation():
    
    rospy.init_node('bluerov_kalmanfilter_node')
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        sub = rospy.Subscriber("bluerov_dvl", DVL, callback)
        rate.sleep()

        
if __name__ == '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
