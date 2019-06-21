#!/usr/bin/env python

# Python script to do obstacle detection with artifical potential field (using the script afp)
import rospy
import numpy as np
import sys
from afp import compute_gradient_direction 

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu


laser_ranges=np.empty(360)
theta_p=0.0

def laser_scan_callback(msg):
	global laser_ranges
	laser_ranges=msg.ranges

def imu_callback(msg):
	global theta_p
	theta_p=msg.angular_velocity.z


def avoid_obstacle():
	global laser_ranges, theta_p
	################## INITIALIZATION ##################
	# Initialize Node
	rospy.init_node('turtlebot3', anonymous=True)
	# Initialize Publisher
	cmd_topic='/cmd_vel'
	twist_publisher=rospy.Publisher(cmd_topic, Twist, queue_size=10)
	# Initialize Twist Message
	twist_msg=Twist()

	# Initialize Laser Scan Subscriber
	sub = rospy.Subscriber('/scan', LaserScan, laser_scan_callback)

	# Initialize Imu Subscriber
	sub = rospy.Subscriber('/imu', Imu, imu_callback)

	# Initialize Rate
	rate=rospy.Rate(10)

	twist_msg.linear.x=0.02
	twist_msg.linear.y=0.0
	twist_msg.linear.z=0.0
	twist_msg.angular.x=0.0
	twist_msg.angular.y=0.0

	while not rospy.is_shutdown():

		desired_theta_p=0.05

		while not (desired_theta_p<0.005)&(desired_theta_p>-0.005):

			# Wait for Laser Scan
			rospy.wait_for_message('/scan', LaserScan)
			laser_ranges=np.asarray(laser_ranges)

			# # Wait for Imu
			# rospy.wait_for_message('/imu', Imu)
			# print(theta_p)

			# Get desired direction
			gradient_direction=compute_gradient_direction(laser_ranges)
			print("I should move my trajectory by: "+str(gradient_direction*180/np.pi)+ " rad.")

			# Compute difference
			K=0.2
			desired_theta_p=K*gradient_direction

			# Apply angular velocity
			
			twist_msg.angular.z=desired_theta_p
			print("I am publishing an angular velocity of: "+str(desired_theta_p)+" rad/s")
			twist_publisher.publish(twist_msg)

			rate.sleep()

		print("Clear path found!")

		
		#sys.exit(1)



if __name__=="__main__":
	try:
		avoid_obstacle()
	except rospy.ROSInterruptException:
		pass