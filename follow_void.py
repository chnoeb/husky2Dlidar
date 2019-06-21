#! /usr/bin/env python

# Python script to run an temporary autopilot.
# The program computes the different possible paths swept by the robot and chooses the path which causes the collision the further possible.

import rospy
import numpy as np

import matplotlib.pyplot as plt
from matplotlib import patches
import time
import threading

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class VoidFollower:
	def __init__(self):

		# Initialize Node
		rospy.init_node('void_follower')

		# Initialize Swtich Subscriber
		self.switch_bool=False
		switch_sub = rospy.Subscriber('/follow_void_switch', Bool, self.switch_callback)

		# Initialize Laser Scan Subscriber
		laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)

		# Initialize Publisher
		self.cmd_topic='/cmd_vel'
		self.twist_publisher=rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
		
		# Initialize Twist Message
		self.v_max=0.1 # Maximum possible linear velocity
		self.omega_max=0.2 # Maximum possible angular velocity
		self.v=self.v_max # Current path linear velocity for a given time
		self.target_v=0.00 # Target linear velocity sent to the robot
		self.target_omega=0.00 # Target angular velocity sent to the robot
		
		self.twist_msg=Twist()
		self.twist_msg.linear.x=self.v
		self.twist_msg.linear.y=0.0
		self.twist_msg.linear.z=0.0
		self.twist_msg.angular.x=0.0
		self.twist_msg.angular.y=0.0
		self.twist_msg.angular.z=0.0

		# Initialize Threading
		self.cmd_lock=threading.RLock()
		self.cmd_threader=threading.Thread(target=self.cmd_thread_function, args=())
		self.cmd_threader.start()

		# Initialize Data
		################## FOR GAZEBO ##################
		# self.laser_scan=np.empty(360) 
		# self.angle_scan=np.linspace(0,2*np.pi,len(self.laser_scan))
		# self.angle_scan[self.angle_scan>np.pi]=self.angle_scan[self.angle_scan>np.pi]-2*np.pi
		################################################

		self.laser_scan=np.empty(811) # For sick_tim, 811 point scan
		self.angle_scan=np.linspace(-2.35619449615,2.35572338104,len(self.laser_scan))

		# Initialize Lidar Plot
		self.fig = plt.figure()
		self.ax_lidar= self.fig.add_subplot(1,1,1)
		plt.gca().set_aspect('equal', adjustable='box')
		self.ax_lidar.set_xlim((-2,2))
		self.ax_lidar.set_ylim((-2,2))
		self.line_length=6.0  # Line in plot length

		# Initialize Parameters
		self.min_path_cone_rad=-45*np.pi/180 # Cone field of view minimum angle
		self.max_path_cone_rad=45*np.pi/180 # Cone field of view maximum angle
		self.min_field_of_view_rad=-135*np.pi/180 # Field of view minimum angle 
		self.max_field_of_view_rad=135*np.pi/180 # Field of view minimum angle 
		self.inf_value=10.0 # If value is 'inf' convert it to inf_value
		
		self.car_width=0.67+0.03
		self.car_length=0.80
		self.stop_distance=0.25+self.car_length/2
		self.slow_distance=0.50+self.car_length/2
	
	def switch_callback(self, msg):
		self.switch_bool=msg.data

	def laser_scan_callback(self, msg):
		self.laser_scan=np.asarray(msg.ranges)

	def follow_void(self):

		# While not shutdown
		while not rospy.is_shutdown():

			while self.switch_bool:
				# Read Scan
				rospy.wait_for_message('/scan', LaserScan)
				# Transform and Scale
				self.transform_laser()
				# Check for obstacles
				self.check_for_obstacles()
				# Plot Lidar (optionnal)
				self.plot_lidar()
				# Move
				self.move()


			# Exiting Program
			self.twist_msg.linear.x=0.0
			self.twist_msg.linear.y=0.0
			self.twist_msg.linear.z=0.0
			self.twist_msg.angular.x=0.0
			self.twist_msg.angular.y=0.0
			self.twist_msg.angular.z=0.0
			self.twist_publisher.publish(self.twist_msg)

			while not self.switch_bool:
				# Read
				rospy.wait_for_message('/scan', LaserScan)
				# Transform and Scale
				self.transform_laser()
				# Check for obstacles
				self.check_for_obstacles()
				# Plot Lidar (optionnal)
				self.plot_lidar()

		


	def transform_laser(self):
		# Convert inf to inf_value
		self.laser_scan[self.laser_scan==np.inf]=self.inf_value
		# Check for near noise values (SICK)
		self.laser_scan[self.laser_scan<0.005]=self.inf_value
		# Keep forward (can be better)
		self.valid_laser=(self.angle_scan<self.max_field_of_view_rad)&(self.angle_scan>self.min_field_of_view_rad)
		# Keep cone (can be better)
		self.valid_cone=(self.angle_scan[self.valid_laser]<self.max_path_cone_rad)&(self.angle_scan[self.valid_laser]>self.min_path_cone_rad)


		# Polar to cartesian
		x_laser=self.laser_scan[self.valid_laser]*np.cos(self.angle_scan[self.valid_laser])
		y_laser=self.laser_scan[self.valid_laser]*np.sin(self.angle_scan[self.valid_laser])
		# Apply Transform (add half car length for Husky because Sick is in front)
		x_laser+=self.car_length/2
		# Set x & y and range & theta arrays
		self.xy_laser=np.array((x_laser, y_laser)).T
		# Recompute radius and theta
		self.rt_laser=np.array(((x_laser**2+y_laser**2)**0.5, np.arctan2(y_laser, x_laser))).T
		

	def find_best_theta(self):
		cone_laser=self.rt_laser_avg[self.valid_cone,:]
		best_idx=np.argmax(cone_laser[:,0])	# Find max range within path hitbox array and within cone
		self.best_direction=cone_laser[best_idx,1] # Get path hitbox max range direction
		self.best_range=cone_laser[best_idx,0] # Get path hitbox max range direction

	def unstuck(self):
		# Suggestion: Should move backwards instead ?
		# IN CONSTRUCTION 

		# # Determine if there is more free space to the left or right of the robot
		# left_sum=np.sum(self.rt_laser[self.rt_laser[:,1]>0,0])
		# right_sum=np.sum(self.rt_laser[self.rt_laser[:,1]<0,0])

		# if (left_sum>right_sum):
		# 	self.best_direction=self.max_path_cone_rad # Go left
		# else:
		# 	self.best_direction=self.min_path_cone_rad # Go right

		# Temporaire
		self.best_direction=0.0
		
		
	def check_for_obstacles(self):

		# Initialize linear speed to v_max
		self.v=self.v_max
		# Apply Car Hitbox Filter
		self.apply_car_hitbox_filter()
		# Apply 1D Filter 
		self.apply_1D_filter()
		# Find Best Direction for v_max speed
		self.find_best_theta()
		
		# Set iteration counter
		iter=0
		
		# Loop for 3 iterations max
		while iter<2:
			iter+=1 # Update iteration counter
			if self.best_range>self.slow_distance: # If direction is valid
				break
			elif self.best_range<self.stop_distance: # If direction is within stop_distance
				self.v=0.00	# Set linear speed to 0.00
				self.apply_car_hitbox_filter()
				self.apply_1D_filter()
				self.find_best_theta()
				
				# Stuck
				if self.best_range<self.stop_distance: # If best direction is still within stop_distance
					print("Possibly Stuck")
					self.unstuck()
					break
			else: # If direction is within slow_distance
				self.v=self.v*(self.best_range-self.stop_distance)/(self.slow_distance-self.stop_distance) # Compute slowed linear speed
				self.apply_car_hitbox_filter()
				# Apply 1D Filter (optionnal)
				self.apply_1D_filter()
				self.find_best_theta()
				

	def apply_1D_filter(self):


		# Moving average
		
		# # Gaussian Average 7 Sigma=1.0
		# laser_scan_avg=(0.006*np.roll(self.laser_scan,-3, axis=0)
		# 			  +0.061*np.roll(self.laser_scan,-2, axis=0)
		# 			  +0.242*np.roll(self.laser_scan,-1, axis=0)
		# 			  +0.006*np.roll(self.laser_scan,3, axis=0)
		# 			  +0.061*np.roll(self.laser_scan,2, axis=0)
		# 			  +0.242*np.roll(self.laser_scan,1, axis=0)
		# 			  +0.383*self.laser_scan)
		
		# # Uniform Average
		# laser_scan_avg=(np.roll(self.laser_scan,-3, axis=0)
		# 			  +np.roll(self.laser_scan,-2, axis=0)
		# 			  +np.roll(self.laser_scan,-1, axis=0)
		# 			  +np.roll(self.laser_scan,3, axis=0)
		# 			  +np.roll(self.laser_scan,2, axis=0)
		# 			  +np.roll(self.laser_scan,1, axis=0)
		# 			  +self.laser_scan)/7
		
		# # Inverse Gaussian Average
		# laser_scan_avg=1/(0.006*np.roll(1/self.laser_scan,-3, axis=0)
		# 			  +0.061*np.roll(1/self.laser_scan,-2, axis=0)
		# 			  +0.242*np.roll(1/self.laser_scan,-1, axis=0)
		# 			  +0.006*np.roll(1/self.laser_scan,3, axis=0)
		# 			  +0.061*np.roll(1/self.laser_scan,2, axis=0)
		# 			  +0.242*np.roll(1/self.laser_scan,1, axis=0)
		# 			  +0.383*1/(self.laser_scan))
		
		# # Inverse Uniform Average
		# laser_scan_avg=1/(np.roll(1/self.laser_scan,-9, axis=0)
		# 			  +np.roll(1/self.laser_scan,-6, axis=0)
		# 			  +np.roll(1/self.laser_scan,-3, axis=0)
		# 			  +np.roll(1/self.laser_scan,9, axis=0)
		# 			  +np.roll(1/self.laser_scan,6, axis=0)
		# 			  +np.roll(1/self.laser_scan,3, axis=0)
		# 			  +1/self.laser_scan)/7
		
		# Inverse Squared Gaussian Average 11 Sigma=1.0
		laser_scan_avg=(0.000003*np.roll(self.rt_laser_rp[:,0]**-2,-15, axis=0)
					  +0.000229*np.roll(self.rt_laser_rp[:,0]**-2,-12, axis=0)
					  +0.005977*np.roll(self.rt_laser_rp[:,0]**-2,-9, axis=0)
					  +0.060598*np.roll(self.rt_laser_rp[:,0]**-2,-6, axis=0)
					  +0.24173*np.roll(self.rt_laser_rp[:,0]**-2,-3, axis=0)
					  +0.000003*np.roll(self.rt_laser_rp[:,0]**-2,15, axis=0)
					  +0.000229*np.roll(self.rt_laser_rp[:,0]**-2,12, axis=0)
					  +0.005977*np.roll(self.rt_laser_rp[:,0]**-2,9, axis=0)
					  +0.060598*np.roll(self.rt_laser_rp[:,0]**-2,6, axis=0)
					  +0.24173*np.roll(self.rt_laser_rp[:,0]**-2,3, axis=0)
					  +0.382925*self.rt_laser_rp[:,0]**-2)**-0.5
		
		# #Inverse Squared Gaussian Average 11 Sigma=3.0
		# laser_scan_avg=(0.035822*np.roll(self.laser_scan**-2,-15, axis=0)
		# 			  +0.058790*np.roll(self.laser_scan**-2,-12, axis=0)
		# 			  +0.086425*np.roll(self.laser_scan**-2,-9, axis=0)
		# 			  +0.113806*np.roll(self.laser_scan**-2,-6, axis=0)
		# 			  +0.134240*np.roll(self.laser_scan**-2,-3, axis=0)
		# 			  +0.035822*np.roll(self.laser_scan**-2,15, axis=0)
		# 			  +0.058790*np.roll(self.laser_scan**-2,12, axis=0)
		# 			  +0.086425*np.roll(self.laser_scan**-2,9, axis=0)
		# 			  +0.113806*np.roll(self.laser_scan**-2,6, axis=0)
		# 			  +0.134240*np.roll(self.laser_scan**-2,3, axis=0)
		# 			  +0.141836*self.laser_scan**-2)**-0.5

		# Compute averaged cartesian coordinates
		x_laser_avg=laser_scan_avg*np.cos(self.rt_laser_rp[:,1])
		y_laser_avg=laser_scan_avg*np.sin(self.rt_laser_rp[:,1])
		# Set averaged x & y and averaged range & theta arrays
		self.xy_laser_avg=np.array((x_laser_avg, y_laser_avg)).T
		self.rt_laser_avg=np.array((laser_scan_avg, self.rt_laser_rp[:,1])).T

	def apply_car_hitbox_filter(self):


		# Compute line parametric equation coefficients
		a=-np.sin(self.rt_laser[:,1]) # Compute line equations coefficient a
		b=np.cos(self.rt_laser[:,1]) # Compute line equations coefficient b


		# Turn Radius Path
		r_c=np.full_like(a, 1000.0) # Initialize turn radius array to the same shape as a
		not_zero_idx=(self.rt_laser[:,1]!=0.0) # Find lidar points which are not directly ahead (theta=0.0)
		r_c[not_zero_idx]=self.v/(np.absolute(self.rt_laser[not_zero_idx,1])*self.omega_max/self.max_path_cone_rad) # Compute turn radius array for non zero thetas
		r_c_m=r_c-self.car_width/2 # Compute minus turn radius
		r_c_p=r_c+self.car_width/2 # Compute plus turn radius

		r_c_back_path=((r_c+self.car_width/2)**2+(self.car_length/2)**2)**0.5 # Compute front wheel turn radius
		r_c_front_path=((r_c+self.car_width/2)**2+(self.car_length/2)**2)**0.5  # Compute back wheel turn radius
		theta_path=np.arctan2(self.car_length/2, r_c+self.car_width/2) # Compute turn front and back wheel turn radius angle
		# Suggestion: Replace if by negative r_c and negative theta?

		# Initialize car path array
		self.rt_laser_rp=self.rt_laser.copy()
		
		for idx in range(len(a)): # Loop for all points within the view (Could be ameliorated by looping on only the cone points)

			if self.rt_laser[idx,1]>=0: # If theta is a left turn, apply left turn equations
				
				# Compute radius and theta from turn center point
				radius=self.xy_laser[:,0]**2+(self.xy_laser[:,1]-r_c[idx])**2
				theta=np.arctan2(self.xy_laser[:,1]-r_c[idx], self.xy_laser[:,0])+np.pi/2

				# Compute c constant for car linear lines (c is the normal displacement from origin)
				c_m=r_c[idx]*(1-b[idx])-self.car_width/2
				c_p=r_c[idx]*(1-b[idx])+self.car_width/2
				c_t=r_c[idx]*a[idx]+self.car_length/2

				line_eq_m=a[idx]*self.xy_laser[:,0]+b[idx]*self.xy_laser[:,1]+c_m # Check if a point is above minus car line
				line_eq_p=a[idx]*self.xy_laser[:,0]+b[idx]*self.xy_laser[:,1]+c_p # Check if a point is below plus car line
				line_eq_t=b[idx]*self.xy_laser[:,0]+-a[idx]*self.xy_laser[:,1]+c_t # Check if a point after the circular path

				in_car_initial=(self.xy_laser[:,0]<=self.car_length/2)&(self.xy_laser[:,0]>=-self.car_length/2)&(self.xy_laser[:,1]<=self.car_width/2)&(self.xy_laser[:,1]>=-self.car_width/2)
				in_back_path=(radius<r_c_back_path[idx]**2)&(radius>r_c_m[idx]**2)&(theta>0.0-theta_path[idx])&(theta<-theta_path[idx]+self.rt_laser[idx,1])
				in_front_path=(radius<r_c_front_path[idx]**2)&(radius>r_c_m[idx]**2)&(theta>0.0+theta_path[idx])&(theta<theta_path[idx]+self.rt_laser[idx,1])
				in_turn_path=in_car_initial|in_back_path|in_front_path
			
			
			else: # If theta is a right turn, apply right turn equations
				
				# Compute radius and theta from turn center point
				radius=self.xy_laser[:,0]**2+(self.xy_laser[:,1]+r_c[idx])**2
				theta=np.arctan2(self.xy_laser[:,1]+r_c[idx], self.xy_laser[:,0])-np.pi/2-self.rt_laser[idx,1]

				# Compute c constant for car linear lines (c is the normal displacement from origin)
				c_m=-r_c[idx]*(1-b[idx])-self.car_width/2
				c_p=-r_c[idx]*(1-b[idx])+self.car_width/2
				c_t=-r_c[idx]*a[idx]+self.car_length/2

				line_eq_m=a[idx]*self.xy_laser[:,0]+b[idx]*self.xy_laser[:,1]+c_m # Check if a point is above minus car line
				line_eq_p=a[idx]*self.xy_laser[:,0]+b[idx]*self.xy_laser[:,1]+c_p # Check if a point is below plus car line
				line_eq_t=b[idx]*self.xy_laser[:,0]+-a[idx]*self.xy_laser[:,1]+c_t # Check if a point after the circular path

				in_car_initial=(self.xy_laser[:,0]<=self.car_length/2)&(self.xy_laser[:,0]>=-self.car_length/2)&(self.xy_laser[:,1]<=self.car_width/2)&(self.xy_laser[:,1]>=-self.car_width/2)
				in_back_path=(radius<r_c_back_path[idx]**2)&(radius>r_c_m[idx]**2)&(theta>0.0+theta_path[idx])&(theta<-self.rt_laser[idx,1]+theta_path[idx])
				in_front_path=(radius<r_c_back_path[idx]**2)&(radius>r_c_m[idx]**2)&(theta>0.0-theta_path[idx])&(theta<-self.rt_laser[idx,1]-theta_path[idx])
				in_turn_path=in_car_initial|in_back_path|in_front_path
			

			in_line_path=(line_eq_m<=0.0)&(line_eq_p>=0.0)&(line_eq_t>=0.0) # Find points within linear car path

			in_turn_or_line_path=(in_turn_path)|(in_line_path) # Find points within car path

			if np.any(in_turn_or_line_path): # If no points were find within the path
				self.rt_laser_rp[idx,0]=np.min(self.rt_laser[in_turn_or_line_path,0])
			else:
				self.rt_laser_rp[idx,0]=self.inf_value


		# Set car path x & y 
		self.xy_laser_rp=np.array((self.rt_laser_rp[:,0]*np.cos(self.rt_laser_rp[:,1]), self.rt_laser_rp[:,0]*np.sin(self.rt_laser_rp[:,1]))).T




			


	def plot_lidar(self):

		self.ax_lidar.clear()

		# Plot robot
		robot_rectangle=patches.Rectangle((-self.car_length/2, -self.car_width/2), self.car_length, self.car_width)
		self.ax_lidar.add_patch(robot_rectangle)
		# Plot Lidar
		self.ax_lidar.plot(self.xy_laser[:,0], self.xy_laser[:, 1], 'o', markerfacecolor='b', markeredgecolor='k', markersize=3) # Plot real Lidar points
		self.ax_lidar.plot(self.xy_laser_avg[:,0], self.xy_laser_avg[:, 1], 'o', markerfacecolor='r', markeredgecolor='k', markersize=3, alpha=0.25) # Plot averaged Lidar points
		self.ax_lidar.plot(self.xy_laser_rp[:,0], self.xy_laser_rp[:, 1], 'o', markerfacecolor='g', markeredgecolor='k', markersize=3, alpha=0.25) # Plot path hitbox Lidar points
		
		# Plot Direction
		self.ax_lidar.plot(np.array((0,self.line_length*np.cos(self.best_direction))), np.array((0,self.line_length*np.sin(self.best_direction))), color='r')

		# Plot Projected Path
		if (self.best_direction!=0.0):
			r_c=self.v/(abs(self.best_direction)*self.omega_max/self.max_path_cone_rad)
		else:
			r_c=1000.0
		
		r_c_m=r_c-self.car_width/2
		r_c_p=r_c+self.car_width/2
		
		r_c_back_path=((r_c+self.car_width/2)**2+(self.car_length/2)**2)**0.5
		r_c_front_path=((r_c+self.car_width/2)**2+(self.car_length/2)**2)**0.5



		if (self.best_direction>=0.0):
			theta_path=np.arctan2(self.car_length/2, r_c+self.car_width/2)
			arc_path_back=patches.Arc((0,r_c), r_c_back_path*2, r_c_back_path*2, angle=-90.0, theta1=0.0-theta_path*180/np.pi, theta2=-theta_path*180/np.pi+self.best_direction*180/np.pi, color='g', linewidth=1)
			arc_path_front=patches.Arc((0,r_c), r_c_front_path*2, r_c_front_path*2, angle=-90.0, theta1=0.0+theta_path*180/np.pi, theta2=theta_path*180/np.pi+self.best_direction*180/np.pi, color='g', linewidth=1)

			arc_bottom=patches.Arc((0,r_c), r_c_m*2, r_c_m*2, angle=-90.0, theta1=0.0, theta2=self.best_direction*180/np.pi, color='g', linewidth=1)
			arc_center=patches.Arc((0,r_c), r_c*2, r_c*2, angle=-90.0, theta1=0.0, theta2=self.best_direction*180/np.pi, color='k', linewidth=1)

			# Red x
			self.ax_lidar.scatter(r_c*np.sin(self.best_direction), r_c*(1-np.cos(self.best_direction)), marker='x', color='r', s=30)
			# Projected robot rectangle
			projected_robot_rectangle=patches.Rectangle((0,0), self.car_length, self.car_width, angle=self.best_direction*180/np.pi, color='r', alpha=0.2)
			projected_robot_rectangle.set_xy((r_c*np.sin(self.best_direction)-np.cos(self.best_direction)*self.car_length/2+np.sin(self.best_direction)*self.car_width/2, r_c*(1-np.cos(self.best_direction))-np.sin(self.best_direction)*self.car_length/2-np.cos(self.best_direction)*self.car_width/2))
			# Green lines
			self.ax_lidar.plot(np.array((r_c*np.sin(self.best_direction), r_c*np.sin(self.best_direction)+self.line_length*np.cos(self.best_direction))), np.array((r_c*(1-np.cos(self.best_direction)), r_c*(1-np.cos(self.best_direction))+self.line_length*np.sin(self.best_direction))), color='k', linewidth=1)
			self.ax_lidar.plot(np.array((r_c_m*np.sin(self.best_direction), r_c_m*np.sin(self.best_direction)+self.line_length*np.cos(self.best_direction))), np.array((r_c-r_c_m*np.cos(self.best_direction), r_c-r_c_m*np.cos(self.best_direction)+self.line_length*np.sin(self.best_direction))), color='g', linewidth=1)
			
			self.ax_lidar.plot(np.array((r_c_p*np.sin(self.best_direction)+np.cos(self.best_direction)*self.car_length/2,
											  r_c_p*np.sin(self.best_direction)+self.line_length*np.cos(self.best_direction)+np.cos(self.best_direction)*self.car_length/2)),
									np.array((r_c-r_c_p*np.cos(self.best_direction)+np.sin(self.best_direction)*self.car_length/2,
											  r_c-r_c_p*np.cos(self.best_direction)+self.line_length*np.sin(self.best_direction)+np.sin(self.best_direction)*self.car_length/2)), color='g', linewidth=1)


		else:
			theta_path=np.arctan2(self.car_length/2, r_c+self.car_width/2)
			arc_path_back=patches.Arc((0,-r_c), r_c_back_path*2, r_c_back_path*2, angle=90.0+self.best_direction*180/np.pi, theta1=0.0+theta_path*180/np.pi, theta2=-self.best_direction*180/np.pi+theta_path*180/np.pi, color='g', linewidth=1)
			arc_path_front=patches.Arc((0,-r_c), r_c_front_path*2, r_c_front_path*2, angle=90.0+self.best_direction*180/np.pi, theta1=0.0-theta_path*180/np.pi, theta2=-self.best_direction*180/np.pi-theta_path*180/np.pi, color='g', linewidth=1)

			arc_bottom=patches.Arc((0,-r_c), r_c_m*2, r_c_m*2, angle=90.0+self.best_direction*180/np.pi, theta1=0.0, theta2=-self.best_direction*180/np.pi, color='g', linewidth=1)
			arc_center=patches.Arc((0,-r_c), r_c*2, r_c*2, angle=90.0+self.best_direction*180/np.pi, theta1=0.0, theta2=-self.best_direction*180/np.pi, color='k', linewidth=1)

			# Red x
			self.ax_lidar.scatter(r_c*np.sin(-self.best_direction), -r_c*(1-np.cos(self.best_direction)), marker='x', color='r', s=30)
			# Projected robot rectangle
			projected_robot_rectangle=patches.Rectangle((0,0), self.car_length, self.car_width, angle=self.best_direction*180/np.pi, color='r', alpha=0.2)
			projected_robot_rectangle.set_xy((r_c*np.sin(-self.best_direction)-np.cos(self.best_direction)*self.car_length/2+np.sin(self.best_direction)*self.car_width/2, -r_c*(1-np.cos(self.best_direction))-np.sin(self.best_direction)*self.car_length/2-np.cos(self.best_direction)*self.car_width/2))
			# Green lines
			self.ax_lidar.plot(np.array((r_c*np.sin(-self.best_direction), r_c*np.sin(-self.best_direction)+self.line_length*np.cos(self.best_direction))), np.array((-r_c*(1-np.cos(self.best_direction)), -r_c*(1-np.cos(self.best_direction))+self.line_length*np.sin(self.best_direction))), color='k', linewidth=1)
			self.ax_lidar.plot(np.array((r_c_m*np.sin(-self.best_direction), r_c_m*np.sin(-self.best_direction)+self.line_length*np.cos(self.best_direction))), np.array((-r_c+r_c_m*np.cos(self.best_direction), -r_c+r_c_m*np.cos(self.best_direction)+self.line_length*np.sin(self.best_direction))), color='g', linewidth=1)
			
			self.ax_lidar.plot(np.array((r_c_p*np.sin(-self.best_direction)+np.cos(self.best_direction)*self.car_length/2,
											  r_c_p*np.sin(-self.best_direction)+self.line_length*np.cos(self.best_direction)+np.cos(self.best_direction)*self.car_length/2)),
									np.array((-r_c+r_c_p*np.cos(self.best_direction)+np.sin(self.best_direction)*self.car_length/2,
											 -r_c+r_c_p*np.cos(self.best_direction)+self.line_length*np.sin(self.best_direction)+np.sin(self.best_direction)*self.car_length/2)), color='g', linewidth=1)

		# Add patches
		self.ax_lidar.add_patch(projected_robot_rectangle)
		if r_c>0.0:
			self.ax_lidar.add_patch(arc_center)
		self.ax_lidar.add_patch(arc_bottom)
		self.ax_lidar.add_patch(arc_path_front)
		self.ax_lidar.add_patch(arc_path_back)

		# Plot Lidar Points within Projected Path Lines (same code as in car_hitbox_filter)
		# Suggestion: Save hitbox points to not run code twice?
		a_direction=-np.sin(self.best_direction)
		b_direction=np.cos(self.best_direction)


		if self.best_direction>=0.0:
			radius=self.xy_laser[:,0]**2+(self.xy_laser[:,1]-r_c)**2
			theta=np.arctan2(self.xy_laser[:,1]-r_c, self.xy_laser[:,0])+np.pi/2

			c_m=r_c*(1-b_direction)-self.car_width/2
			c_p=r_c*(1-b_direction)+self.car_width/2
			c_t=r_c*a_direction+self.car_length/2

			line_eq_m=a_direction*self.xy_laser[:,0]+b_direction*self.xy_laser[:,1]+c_m
			line_eq_p=a_direction*self.xy_laser[:,0]+b_direction*self.xy_laser[:,1]+c_p
			line_eq_t=b_direction*self.xy_laser[:,0]+-a_direction*self.xy_laser[:,1]+c_t

			in_car_initial=(self.xy_laser[:,0]<=self.car_length/2)&(self.xy_laser[:,0]>=-self.car_length/2)&(self.xy_laser[:,1]<=self.car_width/2)&(self.xy_laser[:,1]>=-self.car_width/2)
			in_back_path=(radius<r_c_back_path**2)&(radius>r_c_m**2)&(theta>0.0-theta_path)&(theta<-theta_path+self.best_direction)
			in_front_path=(radius<r_c_front_path**2)&(radius>r_c_m**2)&(theta>0.0+theta_path)&(theta<theta_path+self.best_direction)
			in_turn_path=in_car_initial|in_back_path|in_front_path

			

		else:
			radius=self.xy_laser[:,0]**2+(self.xy_laser[:,1]+r_c)**2
			theta=np.arctan2(self.xy_laser[:,1]+r_c, self.xy_laser[:,0])-np.pi/2-self.best_direction

			c_m=-r_c*(1-b_direction)-self.car_width/2
			c_p=-r_c*(1-b_direction)+self.car_width/2
			c_t=-r_c*a_direction+self.car_length/2

			line_eq_m=a_direction*self.xy_laser[:,0]+b_direction*self.xy_laser[:,1]+c_m
			line_eq_p=a_direction*self.xy_laser[:,0]+b_direction*self.xy_laser[:,1]+c_p
			line_eq_t=b_direction*self.xy_laser[:,0]+-a_direction*self.xy_laser[:,1]+c_t

			in_car_initial=(self.xy_laser[:,0]<=self.car_length/2)&(self.xy_laser[:,0]>=-self.car_length/2)&(self.xy_laser[:,1]<=self.car_width/2)&(self.xy_laser[:,1]>=-self.car_width/2)
			in_back_path=(radius<r_c_back_path**2)&(radius>r_c_m**2)&(theta>0.0+theta_path)&(theta<-self.best_direction+theta_path)
			in_front_path=(radius<r_c_back_path**2)&(radius>r_c_m**2)&(theta>0.0-theta_path)&(theta<-self.best_direction-theta_path)
			in_turn_path=in_car_initial|in_back_path|in_front_path

		
		in_line_path=(line_eq_m<=0.0)&(line_eq_p>=0.0)&(line_eq_t>=0.0) # Verify for t

		in_turn_or_line_path=(in_turn_path)|(in_line_path)

		if np.any(in_turn_or_line_path):
			self.ax_lidar.plot(self.xy_laser[in_turn_or_line_path,0], self.xy_laser[in_turn_or_line_path, 1], 'o', markerfacecolor='m', markeredgecolor='k', markersize=3)
		else:
			pass

		# Plot Field of View Cone
		self.ax_lidar.plot(self.line_length*np.array((0,np.cos(self.max_path_cone_rad))), self.line_length*np.array((0,np.sin(self.max_path_cone_rad))), color='k')
		self.ax_lidar.plot(self.line_length*np.array((0,np.cos(self.min_path_cone_rad))), self.line_length*np.array((0,np.sin(self.min_path_cone_rad))), color='k')
		self.ax_lidar.set_xlim((-1,3))
		self.ax_lidar.set_ylim((-3,3))
		# self.ax_lidar.set_xlim((-0.25,0.75))
		# self.ax_lidar.set_ylim((-0.75,0.75))

		plt.show(block=False)
		plt.pause(0.0001)

	def move(self):

		with self.cmd_lock:
			K=self.omega_max/self.max_path_cone_rad # Suggestion: Should be a parameter?
			self.target_omega=K*self.best_direction # Find the desired angular speed according to the best direction
			self.target_v=self.v
	
	def cmd_thread_function(self):
		while not rospy.is_shutdown():
			while self.switch_bool:
				# Set Twist Message
				self.twist_msg.angular.z=self.target_omega
				self.twist_msg.linear.x=self.target_v
				print("Linear velocity: " + str(self.target_v)+" m/s.\t Angular velocity: " + str(self.target_omega) + " rad/s.")
				self.twist_publisher.publish(self.twist_msg)
				time.sleep(0.01)


if __name__=='__main__':
	try:
		void_follower=VoidFollower()
		void_follower.follow_void()
	except rospy.ROSInterruptException:
		pass