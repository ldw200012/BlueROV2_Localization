#!/usr/bin/env python

# Topic Names to be Published and Subscribed
ROS_SUBSCRIBER_NAME = 'dvl/data'
ROS_PUBLISHER_NAME = 'bluerov2/nav'

# Packages to be imported
import rospy
import json
import time
import numpy as np
# we need time, vx, vy, vz, fom, valid
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from bluerov2_kalmanfilter.msg import DVL
from numpy.linalg import inv

class KalmanFilter:
    
    def __init__(self, P):
        self.x = None
        self.z = None
        self.u = None
        self.P = None
        self.K = None
        self.R_base = np.eye(4) # To be multiplied with the sensor uncertainty
        self.R = None
        self.I = np.eye(9)
        self.G = np.transpose(np.array([[0, 0, 0, 0, 0, 0, 0.5, 1, 0]])) # Control Matrix
        self.F = np.array([[1, dt, 0.5*(dt**2), 0, 0, 0, 0, 0, 0], # State Transition Matrix
                           [0, 1, dt, 0,  0,           0, 0,  0,           0],
                           [0, 0,  1, 0,  0,           0, 0,  0,           0],
                           [0, 0,  0, 1, dt, 0.5*(dt**2), 0,  0,           0],
                           [0, 0,  0, 0,  1,          dt, 0,  0,           0],
                           [0, 0,  0, 0,  0,           1, 0,  0,           0],
                           [0, 0,  0, 0,  0,           0, 1, dt, 0.5*(dt**2)],
                           [0, 0,  0, 0,  0,           0, 0,  1,          dt],
                           [0, 0,  0, 0,  0,           0, 0,  0,           1]])
        self.H = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0], # Observation Matrix
                           [0, 0, 0, 0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1, 0]])
        self.Q = np.array([[0.25*(dt**4), 0.5*(dt**3), 0.5*(dt**2), 0., 0., 0., 0., 0., 0.], # Process Noise Uncertainty
                           [0.5*(dt**3), dt**2, dt, 0., 0., 0., 0., 0., 0.],
                           [0.5*(dt**2), dt, 1., 0., 0., 0., 0., 0., 0.],
                           [0., 0., 0., 0.25*(dt**4), 0.5*(dt**3), 0.5*(dt**2), 0., 0., 0.],
                           [0., 0., 0., 0.5*(dt**3), dt**2, dt, 0., 0., 0.],
                           [0., 0., 0., 0.5*(dt**2), dt, 1., 0., 0., 0.],
                           [0., 0., 0., 0., 0., 0., 0.25*(dt**4), 0.5*(dt**3), 0.5*(dt**2)],
                           [0., 0., 0., 0., 0., 0., 0.5*(dt**3), dt**2, dt],
                           [0., 0., 0., 0., 0., 0., 0.5*(dt**2), dt, 1.]])
        self.current_timestamp_sec = None
        self.state_initialized = False
        self.pub = rospy.Publisher(ROS_PUBLISHER_NAME, Odometry, queue_size=10)

    def subscriber(self):
        rospy.Subscriber(ROS_SUBSCRIBER_NAME, DVL, self.time_update)

    def time_update(self, data): # Time Update ("Predict")

        if data.velocity_valid:
            if self.current_timestamp_sec == None:
                self.current_timestamp_sec = data.header.stamp.sec

            elif not self.state_initialized:
                dt = data.header.stamp.sec - self.current_timestamp_sec
                
                x_pos = 0
                y_pos = 0
                z_pos = data.altitude
                x_vel = data.velocity.x
                y_vel = data.velocity.y
                z_vel = data.velocity.z
                x_acc = x_vel / dt
                y_acc = y_vel / dt
                z_acc = z_vel / dt
                self.x = np.array([[x_pos], [x_vel], [x_acc], [y_pos], [y_vel], [y_acc], [z_pos], [z_vel], [z_acc]])
                
                self.state_initialized = True
                self.current_timestamp_sec = data.header.stamp.sec

            else:
                # (1) Project the state ahead
                self.x = (self.F @ self.x) + (self.G @ self.u)

                # (2) Project the error covariance ahead
                self.P = (self.F @ self.P @ self.F.transpose()) + self.Q
            
                measurement_update(data)

    def measurement_update(self, data): # Measurement Update ("Correct")

            # New Measurements
            x_vel = data.velocity.x
            y_vel = data.velocity.y
            z_pos = data.altitude
            z_vel = data.velocity.z
            self.z = [[x_vel], [y_vel], [z_pos], [z_vel]]

            # New Time
            dt = self.current_timestamp_sec - data.header.stamp.sec
            self.current_timestamp_sec = data.header.stamp.sec
            
            # Sensor Measurement Uncertainty
            measurement_uncertainty = data.fom
            self.R = self.R_base * (dt*measurement_uncertainty)

            # (1) Compute the Kalman Gain
            self.K = self.P @ self.H.transpose() @ inv(self.H @ self.P @ self.H.transpose() + self.R)

            # (2) Update Estimate with Measurement z
            self.x = self.x + self.K@(self.z - (self.H@self.x))

            # (3) Update Error Covariance
            self.P = (self.I - (self.K@self.H)) @ self.P
            # self.P = (self.I - (K@self.H)) @ self.P @np.transpose(I - K@H) + K@R@np.transpose(K)

            # Output Odometry message
            bluerov_odom = Odometry()

            bluerov_odom.header.seq = data.header.seq
            bluerov_odom.header.stamp = data.header.stamp
            bluerov_odom.header.frame_id = data.header.frame_id + "_parent"

            bluerov_odom.child_frame_id = data.header.frame_id

            bluerov_odom.pose.pose.position.x = x[0][0]
            bluerov_odom.pose.pose.position.y = x[3][0]
            bluerov_odom.pose.pose.position.z = x[6][0]
            bluerov_odom.pose.pose.orientation.x = None
            bluerov_odom.pose.pose.orientation.y = None
            bluerov_odom.pose.pose.orientation.z = None
            bluerov_odom.pose.pose.orientation.w = None
            bluerov_odom.pose.covariance = None

            bluerov_odom.twist.twist.linear.x = None
            bluerov_odom.twist.twist.linear.y = None
            bluerov_odom.twist.twist.linear.z = None
            bluerov_odom.twist.twist.angular.x = None
            bluerov_odom.twist.twist.angular.y = None
            bluerov_odom.twist.twist.angular.z = None
            bluerov_odom.twist.covariance = None

            self.pub.publish(bluerov_odom)
        
if __name__ == '__main__':

    rospy.init_node('bluerov_kalmanfilter_node')

    BlueROV2_KMF = KalmanFilter()
    BlueROV2_KMF.P = np.eye(9)*500      # P : Estimate Uncertainty
    BlueROV2_KMF.u = np.array([[0]])    # u : Gain Matrix
    BlueROV2_KMF.prev_time_sec = time.time()
    
    try:
    	BlueROV2_KMF.subscriber()
    	rospy.spin()
    except rospy.ROSInterruptException:
        pass