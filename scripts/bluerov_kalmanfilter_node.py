#!/usr/bin/env python

import rospy
import json
import numpy as np
# from bluerov_kalmanfilter.msg import DVL
# we need time, vx, vy, vz, fom, valid
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from numpy.linalg import inv

axis_num = 3
attribute_num = 3
uncertainty = 500
acceleration_z = -9.8
rand_variance_acceleration = 0.1

saved_child_frame_id = "None"

def callback(data):
    
    # Kalman Filter here    
    
    # Initial Estimation
    x = np.zeros((axis_num*attribute_num, 1))
    P = np.eye(axis_num*attribute_num)
    P *= uncertainty
    
    # Time Update ("Predict")
    millisec_to_sec = 1.0/1000.0
    dt = time * millisec_to_sec
    F = np.array([[1., dt, 0.5*(dt**2), 0., 0., 0., 0., 0., 0.],
                    [0., 1, dt, 0., 0., 0., 0., 0., 0.],
                    [0., 0., 1., 0., 0., 0., 0., 0., 0.],
                    [0., 0., 0., 1., dt, 0.5*(dt**2), 0., 0., 0.],
                    [0., 0., 0., 0., 1., dt, 0., 0., 0.],
                    [0., 0., 0., 0., 0., 1., 0., 0., 0.],
                    [0., 0., 0., 0., 0., 0., 1., dt, 0.5*(dt**2)],
                    [0., 0., 0., 0., 0., 0., 0., 1., dt],
                    [0., 0., 0., 0., 0., 0., 0., 0., 1.]])
    G = np.transpose(np.array([[0., 0., 0., 0., 0., 0., 0.5, 1, 0]]))
    u = np.array([[acceleration_z]])
    Q = np.array([[0.25*(dt**4), 0.5*(dt**3), 0.5*(dt**2), 0., 0., 0., 0., 0., 0.],
                  [0.5*(dt**3), dt**2, dt, 0., 0., 0., 0., 0., 0.],
                  [0.5*(dt**2), dt, 1., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0.25*(dt**4), 0.5*(dt**3), 0.5*(dt**2), 0., 0., 0.],
                  [0., 0., 0., 0.5*(dt**3), dt**2, dt, 0., 0., 0.],
                  [0., 0., 0., 0.5*(dt**2), dt, 1., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0.25*(dt**4), 0.5*(dt**3), 0.5*(dt**2)],
                  [0., 0., 0., 0., 0., 0., 0.5*(dt**3), dt**2, dt],
                  [0., 0., 0., 0., 0., 0., 0.5*(dt**2), dt, 1.]])
    
    x = F@x + G@u
    P = F@P@np.transpose(F) + Q
    
    # Measurement Update ("Correct")
    H = np.array([[1., 0., 0., 0., 0., 0., 0., 0., 0.],
                    [0., 0., 0., 1., 0., 0., 0., 0., 0.],
                    [0., 0., 0., 0., 0., 0., 1., 0., 0.]])
    I = np.eye(axis_num*attribute_num)
    R = np.eye(axis_num)
    R *= dt*fom
    z = np.transpose(np.array([[vx*time, vy*time, vz*time]]))
    
    K = P@np.transpose(H)@inv(H@P@np.transpose(H)+R)
    x = x + K@(z - H@x)
    P = (I - K@H)@P@np.transpose(I - K@H) + K@R@np.transpose(K)
    
    # Output Odometry message
    bluerov_odom = Odometry()
    
    bluerov_odom.header.seq = 
    bluerov_odom.header.stamp = 
    bluerov_odom.header.frame_id = 
    
    bluerov_odom.header.child_frame_id = 
    saved_child_frame_id = 
    
    bluerov_odom.pose.position.x = x[0][0]
    bluerov_odom.pose.position.y = x[3][0]
    bluerov_odom.pose.position.z = x[6][0]
    
#     bluerov_odom.pose.orientation.x = 
#     bluerov_odom.pose.orientation.y = 
#     bluerov_odom.pose.orientation.z = 
#     bluerov_odom.pose.orientation.w = 
    
#     bluerov_odom.pose.covariance = 
    
    bluerov_odom.twist.linear.x = x[0][1]
    bluerov_odom.twist.linear.y = x[3][1]
    bluerov_odom.twist.linear.z = x[6][1]
    
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
