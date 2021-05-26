#!/usr/bin/env python

import rospy
import json
import time
import numpy as np
# from bluerov_kalmanfilter.msg import DVL
# we need time, vx, vy, vz, fom, valid
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from bluerov2_kalmanfilter.msg import DVL
from numpy.linalg import inv

class KalmanFilter:
    
    def __init__(self,x, P, u):
        self.x = x
        self.u = u
        self.P = P
        self.time_i = round(time.time() * 1000)
        self.pub = rospy.Publisher('bluerov2/odom', Odometry, queue_size=10)
        
    def time_update(self, frameTimeDiff):
        # ------------------------------------- Time Update ("Predict") ------------------------------------------------
        millisec_to_sec = 1.0/1000.0
        dt = frameTimeDiff * millisec_to_sec

        # State Transition Matrix
        F = np.array([[1., dt, 0.5*(dt**2), 0., 0., 0., 0., 0., 0.],
                        [0., 1, dt, 0., 0., 0., 0., 0., 0.],
                        [0., 0., 1., 0., 0., 0., 0., 0., 0.],
                        [0., 0., 0., 1., dt, 0.5*(dt**2), 0., 0., 0.],
                        [0., 0., 0., 0., 1., dt, 0., 0., 0.],
                        [0., 0., 0., 0., 0., 1., 0., 0., 0.],
                        [0., 0., 0., 0., 0., 0., 1., dt, 0.5*(dt**2)],
                        [0., 0., 0., 0., 0., 0., 0., 1., dt],
                        [0., 0., 0., 0., 0., 0., 0., 0., 1.]])

        # Control Matrix
        G = np.transpose(np.array([[0., 0., 0., 0., 0., 0., 0.5, 1, 0]]))

        # Gain Matrix
        # self.u from parameter

        # Process Noise Uncertainty
        Q = np.array([[0.25*(dt**4), 0.5*(dt**3), 0.5*(dt**2), 0., 0., 0., 0., 0., 0.],
                      [0.5*(dt**3), dt**2, dt, 0., 0., 0., 0., 0., 0.],
                      [0.5*(dt**2), dt, 1., 0., 0., 0., 0., 0., 0.],
                      [0., 0., 0., 0.25*(dt**4), 0.5*(dt**3), 0.5*(dt**2), 0., 0., 0.],
                      [0., 0., 0., 0.5*(dt**3), dt**2, dt, 0., 0., 0.],
                      [0., 0., 0., 0.5*(dt**2), dt, 1., 0., 0., 0.],
                      [0., 0., 0., 0., 0., 0., 0.25*(dt**4), 0.5*(dt**3), 0.5*(dt**2)],
                      [0., 0., 0., 0., 0., 0., 0.5*(dt**3), dt**2, dt],
                      [0., 0., 0., 0., 0., 0., 0.5*(dt**2), dt, 1.]])

        self.x = F@self.x + G@self.u
        self.P = F@self.P@np.transpose(F) + Q
    
    def measurement_update(self, z, frameTimeDiff, measurement_uncertainty, prev_header):
        
        self.time_update(frameTimeDiff)
        
        millisec_to_sec = 1.0/1000.0
        dt = frameTimeDiff * millisec_to_sec
        
        # ---------------------------------- Measurement Update ("Correct") ---------------------------------------------
        # Observation Matrix
        H = np.array([[0., 1., 0., 0., 0., 0., 0., 0., 0.],
                      [0., 0., 0., 0., 1., 0., 0., 0., 0.],
                      [0., 0., 0., 0., 0., 0., 1., 0., 0.],
                      [0., 0., 0., 0., 0., 0., 0., 1., 0.]])

        # Identity Matrix
        I = np.eye(9)

        # Measurement Equation
        # z from parameter

        # Measurement Uncertainty
        R = np.eye(4)
        R *= dt*measurement_uncertainty

        # Kalman Gain
        K = self.P@np.transpose(H)@inv(H@self.P@np.transpose(H)+R)

        self.x = self.x + K@(z - H@x)
        self.P = (I - K@H)@self.P@np.transpose(I - K@H) + K@R@np.transpose(K)

        # publish
        # Output Odometry message
	    bluerov_odom = Odometry()
	    
	    bluerov_odom.header.seq = prev_header[0]
	    bluerov_odom.header.stamp = prev_header[1]
	    bluerov_odom.header.child_frame_id = prev_header[2]
	    
	    bluerov_odom.pose.position.x = x[0][0]
	    bluerov_odom.pose.position.y = x[3][0]
	    bluerov_odom.pose.position.z = x[6][0]
	    
	    bluerov_odom.twist.linear.x = x[1][0]
	    bluerov_odom.twist.linear.y = x[4][0]
	    bluerov_odom.twist.linear.z = x[7][0]

        self.pub.publish(bluerov_odom)
        self.time_i = round(bluerov_odom.header.stamp.sec * 1000)
        return self.x, self.P

    def callback(self, data):
    	z = [data.bi_x_axis, data.bi_y_axis, data.depth, data.bi_z_axis]
    	frameTimeDiff = round(data.header.stamp.sec * 1000) - self.time_i
    	measurement_uncertainty = data.bi_error
    	prev_header = [data.header.seq, data.header.stamp, data.header.frame_id]

    	self.measurement_update(z, frameTimeDiff, measurement_uncertainty, prev_header)

	def subscriber(self):
		rospy.Subscriber("bluerov2/dvl", DVL, self.callback)
        
if __name__ == '__main__':

	rospy.init_node('bluerov_kalmanfilter_node')
	
	# Initial State Vector [0 ... 0]
	x = np.zeros((9, 1))
	
	# Estimate Uncertainty
	P = np.eye(9)*500

	# Gain Matrix
	u = np.array([[0]])

	try:
    	MyKF = KalmanFilter(x, p, u)
    	MyKF.subscriber()
    	rospy.spin()
	except rospy.ROSInterruptException:
	    pass
