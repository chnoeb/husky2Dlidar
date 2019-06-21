#! /usr/bin/env python

# Python script to test the car_hitbox_filter in the follow_void.py program

import numpy as np
import rospy

import matplotlib.pyplot as plt
from matplotlib import patches

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

msg = """
Swept Path!
---------------------------
Change path direction with left and right arrow

CTRL-C to quit
"""

e = """
Communications Failed
"""
MIN_PATH_ANGLE_DEG=-45.0
MAX_PATH_ANGLE_DEG=45.0
THETA_DEG_INC=1.0

MIN_PATH_SPEED=0.0
MAX_PATH_SPEED=0.5
SPEED_INC=0.01

def pathMsg(target_theta_deg, target_speed):
    return "currently:\ttheta %s\t speed %s" % (target_theta_deg, target_speed)

def getKey():

    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output


def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkPathLimitAngle(theta):
    theta = constrain(theta, MIN_PATH_ANGLE_DEG, MAX_PATH_ANGLE_DEG)
    
    return theta

def checkPathLimitSpeed(v):
    v = constrain(v, MIN_PATH_SPEED, MAX_PATH_SPEED)
    
    return v


class PathCalculator:
	def __init__(self):

		# Initialize Lidar Plot
		self.fig = plt.figure()
		self.ax_swept_path= self.fig.add_subplot(1,1,1)
		plt.gca().set_aspect('equal', adjustable='box')
		self.ax_swept_path.set_xlim((-2,2))
		self.ax_swept_path.set_ylim((-2,2))

		# Initialize Parameters
		self.v=0.1
		self.omega_max=0.2
		self.min_path_cone_rad=-45*np.pi/180
		self.max_path_cone_rad=45*np.pi/180
		self.inf_value=10.0
		self.line_length=6.0
		self.car_width=0.67+0.03
		self.car_length=0.95+0.05

		# Initialize Plot
		x=np.linspace(-1.0,3.0,21)
		y=np.linspace(-3.0,3.0,31)
		x_grid, y_grid = np.meshgrid(x,y)

		x_laser=x_grid.flatten()
		y_laser=y_grid.flatten()

		# Set x & y and range & theta arrays
		self.xy_laser=np.array((x_laser, y_laser)).T
		
		

	def plot_car_line(self,target_theta_rad):

		self.ax_swept_path.clear()

		# Plot robot
		robot_rectangle=patches.Rectangle((-self.car_length/2, -self.car_width/2), self.car_length, self.car_width, alpha=0.2)
		self.ax_swept_path.add_patch(robot_rectangle)
		# Plot Grid
		self.ax_swept_path.plot(self.xy_laser[:,0], self.xy_laser[:, 1], 'o', markerfacecolor='b', markeredgecolor='k', markersize=3) # Plot real Lidar points
		

		# Plot Projected Path
		if (target_theta_rad!=0.0):
			r_c=self.v/(abs(target_theta_rad)*self.omega_max/self.max_path_cone_rad)
		else:
			r_c=1000.0
		
		r_c_m=r_c-self.car_width/2
		
		r_c_p=r_c+self.car_width/2
		
		r_c_back_path=((r_c+self.car_width/2)**2+(self.car_length/2)**2)**0.5
		r_c_front_path=((r_c+self.car_width/2)**2+(self.car_length/2)**2)**0.5


		if (target_theta_rad>=0.0):
			#arc_top=patches.Arc((0,r_c), r_c_p*2, r_c_p*2, angle=-90.0, theta1=0.0, theta2=target_theta_rad*180/np.pi, color='g', linewidth=1)
			theta_path=np.arctan2(self.car_length/2, r_c+self.car_width/2)
			arc_path_back=patches.Arc((0,r_c), r_c_back_path*2, r_c_back_path*2, angle=-90.0, theta1=0.0-theta_path*180/np.pi, theta2=-theta_path*180/np.pi+target_theta_rad*180/np.pi, color='g', linewidth=1)
			arc_path_front=patches.Arc((0,r_c), r_c_front_path*2, r_c_front_path*2, angle=-90.0, theta1=0.0+theta_path*180/np.pi, theta2=theta_path*180/np.pi+target_theta_rad*180/np.pi, color='g', linewidth=1)

			arc_bottom=patches.Arc((0,r_c), r_c_m*2, r_c_m*2, angle=-90.0, theta1=0.0, theta2=target_theta_rad*180/np.pi, color='g', linewidth=1)
			arc_center=patches.Arc((0,r_c), r_c*2, r_c*2, angle=-90.0, theta1=0.0, theta2=target_theta_rad*180/np.pi, color='k', linewidth=1)

			# Red x
			self.ax_swept_path.scatter(r_c*np.sin(target_theta_rad), r_c*(1-np.cos(target_theta_rad)), marker='x', color='r', s=30)
			projected_robot_rectangle=patches.Rectangle((0,0), self.car_length, self.car_width, angle=target_theta_rad*180/np.pi, color='r', alpha=0.2)
			projected_robot_rectangle.set_xy((r_c*np.sin(target_theta_rad)-np.cos(target_theta_rad)*self.car_length/2+np.sin(target_theta_rad)*self.car_width/2, r_c*(1-np.cos(target_theta_rad))-np.sin(target_theta_rad)*self.car_length/2-np.cos(target_theta_rad)*self.car_width/2))
			# Green lines
			self.ax_swept_path.plot(np.array((r_c*np.sin(target_theta_rad), r_c*np.sin(target_theta_rad)+self.line_length*np.cos(target_theta_rad))), np.array((r_c*(1-np.cos(target_theta_rad)), r_c*(1-np.cos(target_theta_rad))+self.line_length*np.sin(target_theta_rad))), color='k', linewidth=1)
			self.ax_swept_path.plot(np.array((r_c_m*np.sin(target_theta_rad), r_c_m*np.sin(target_theta_rad)+self.line_length*np.cos(target_theta_rad))), np.array((r_c-r_c_m*np.cos(target_theta_rad), r_c-r_c_m*np.cos(target_theta_rad)+self.line_length*np.sin(target_theta_rad))), color='g', linewidth=1)
			
			self.ax_swept_path.plot(np.array((r_c_p*np.sin(target_theta_rad)+np.cos(target_theta_rad)*self.car_length/2,
											  r_c_p*np.sin(target_theta_rad)+self.line_length*np.cos(target_theta_rad)+np.cos(target_theta_rad)*self.car_length/2)),
									np.array((r_c-r_c_p*np.cos(target_theta_rad)+np.sin(target_theta_rad)*self.car_length/2,
											  r_c-r_c_p*np.cos(target_theta_rad)+self.line_length*np.sin(target_theta_rad)+np.sin(target_theta_rad)*self.car_length/2)), color='g', linewidth=1)


		else:
			theta_path=np.arctan2(self.car_length/2, r_c+self.car_width/2)
			#arc_top=patches.Arc((0,-r_c), r_c_p*2, r_c_p*2, angle=90.0+target_theta_rad*180/np.pi, theta1=0.0, theta2=-target_theta_rad*180/np.pi, color='g', linewidth=1)
			arc_path_back=patches.Arc((0,-r_c), r_c_back_path*2, r_c_back_path*2, angle=90.0+target_theta_rad*180/np.pi, theta1=0.0+theta_path*180/np.pi, theta2=-target_theta_rad*180/np.pi+theta_path*180/np.pi, color='g', linewidth=1)
			arc_path_front=patches.Arc((0,-r_c), r_c_front_path*2, r_c_front_path*2, angle=90.0+target_theta_rad*180/np.pi, theta1=0.0-theta_path*180/np.pi, theta2=-target_theta_rad*180/np.pi-theta_path*180/np.pi, color='g', linewidth=1)

			arc_bottom=patches.Arc((0,-r_c), r_c_m*2, r_c_m*2, angle=90.0+target_theta_rad*180/np.pi, theta1=0.0, theta2=-target_theta_rad*180/np.pi, color='g', linewidth=1)
			arc_center=patches.Arc((0,-r_c), r_c*2, r_c*2, angle=90.0+target_theta_rad*180/np.pi, theta1=0.0, theta2=-target_theta_rad*180/np.pi, color='k', linewidth=1)

			# Red x
			self.ax_swept_path.scatter(r_c*np.sin(-target_theta_rad), -r_c*(1-np.cos(target_theta_rad)), marker='x', color='r', s=30)
			projected_robot_rectangle=patches.Rectangle((0,0), self.car_length, self.car_width, angle=target_theta_rad*180/np.pi, color='r', alpha=0.2)
			projected_robot_rectangle.set_xy((r_c*np.sin(-target_theta_rad)-np.cos(target_theta_rad)*self.car_length/2+np.sin(target_theta_rad)*self.car_width/2, -r_c*(1-np.cos(target_theta_rad))-np.sin(target_theta_rad)*self.car_length/2-np.cos(target_theta_rad)*self.car_width/2))
			# Green lines
			self.ax_swept_path.plot(np.array((r_c*np.sin(-target_theta_rad), r_c*np.sin(-target_theta_rad)+self.line_length*np.cos(target_theta_rad))), np.array((-r_c*(1-np.cos(target_theta_rad)), -r_c*(1-np.cos(target_theta_rad))+self.line_length*np.sin(target_theta_rad))), color='k', linewidth=1)
			self.ax_swept_path.plot(np.array((r_c_m*np.sin(-target_theta_rad), r_c_m*np.sin(-target_theta_rad)+self.line_length*np.cos(target_theta_rad))), np.array((-r_c+r_c_m*np.cos(target_theta_rad), -r_c+r_c_m*np.cos(target_theta_rad)+self.line_length*np.sin(target_theta_rad))), color='g', linewidth=1)
			
			self.ax_swept_path.plot(np.array((r_c_p*np.sin(-target_theta_rad)+np.cos(target_theta_rad)*self.car_length/2,
											  r_c_p*np.sin(-target_theta_rad)+self.line_length*np.cos(target_theta_rad)+np.cos(target_theta_rad)*self.car_length/2)),
									np.array((-r_c+r_c_p*np.cos(target_theta_rad)+np.sin(target_theta_rad)*self.car_length/2,
											 -r_c+r_c_p*np.cos(target_theta_rad)+self.line_length*np.sin(target_theta_rad)+np.sin(target_theta_rad)*self.car_length/2)), color='g', linewidth=1)

		self.ax_swept_path.add_patch(projected_robot_rectangle)
		if r_c>0.0:
			self.ax_swept_path.add_patch(arc_center)
		self.ax_swept_path.add_patch(arc_bottom)
		self.ax_swept_path.add_patch(arc_path_front)
		self.ax_swept_path.add_patch(arc_path_back)


		# Plot Lidar Points within Projected Path Lines
		a_direction=-np.sin(target_theta_rad)
		b_direction=np.cos(target_theta_rad)


		if target_theta_rad>=0.0:
			radius=self.xy_laser[:,0]**2+(self.xy_laser[:,1]-r_c)**2
			theta=np.arctan2(self.xy_laser[:,1]-r_c, self.xy_laser[:,0])+np.pi/2

			c_m=r_c*(1-b_direction)-self.car_width/2
			c_p=r_c*(1-b_direction)+self.car_width/2
			c_t=r_c*a_direction+self.car_length/2

			line_eq_m=a_direction*self.xy_laser[:,0]+b_direction*self.xy_laser[:,1]+c_m
			line_eq_p=a_direction*self.xy_laser[:,0]+b_direction*self.xy_laser[:,1]+c_p
			line_eq_t=b_direction*self.xy_laser[:,0]+-a_direction*self.xy_laser[:,1]+c_t

			in_car_initial=(self.xy_laser[:,0]<=self.car_length/2)&(self.xy_laser[:,0]>=-self.car_length/2)&(self.xy_laser[:,1]<=self.car_width/2)&(self.xy_laser[:,1]>=-self.car_width/2)
			in_back_path=(radius<r_c_back_path**2)&(radius>r_c_m**2)&(theta>0.0-theta_path)&(theta<-theta_path+target_theta_rad)
			in_front_path=(radius<r_c_front_path**2)&(radius>r_c_m**2)&(theta>0.0+theta_path)&(theta<theta_path+target_theta_rad)
			in_turn_path=in_car_initial|in_back_path|in_front_path

			

		else:
			radius=self.xy_laser[:,0]**2+(self.xy_laser[:,1]+r_c)**2
			theta=np.arctan2(self.xy_laser[:,1]+r_c, self.xy_laser[:,0])-np.pi/2-target_theta_rad

			c_m=-r_c*(1-b_direction)-self.car_width/2
			c_p=-r_c*(1-b_direction)+self.car_width/2
			c_t=-r_c*a_direction+self.car_length/2

			line_eq_m=a_direction*self.xy_laser[:,0]+b_direction*self.xy_laser[:,1]+c_m
			line_eq_p=a_direction*self.xy_laser[:,0]+b_direction*self.xy_laser[:,1]+c_p
			line_eq_t=b_direction*self.xy_laser[:,0]+-a_direction*self.xy_laser[:,1]+c_t

			in_car_initial=(self.xy_laser[:,0]<=self.car_length/2)&(self.xy_laser[:,0]>=-self.car_length/2)&(self.xy_laser[:,1]<=self.car_width/2)&(self.xy_laser[:,1]>=-self.car_width/2)
			in_back_path=(radius<r_c_back_path**2)&(radius>r_c_m**2)&(theta>0.0+theta_path)&(theta<-target_theta_rad+theta_path)
			in_front_path=(radius<r_c_back_path**2)&(radius>r_c_m**2)&(theta>0.0-theta_path)&(theta<-target_theta_rad-theta_path)
			in_turn_path=in_car_initial|in_back_path|in_front_path

		
		in_line_path=(line_eq_m<=0.0)&(line_eq_p>=0.0)&(line_eq_t>=0.0) # Verify for t

		in_turn_or_line_path=(in_turn_path)|(in_line_path)

		if np.any(in_turn_or_line_path):
			self.ax_swept_path.plot(self.xy_laser[in_turn_or_line_path,0], self.xy_laser[in_turn_or_line_path, 1], 'o', markerfacecolor='m', markeredgecolor='k', markersize=3)
		else:
			pass

		# Plot Field of View Cone
		self.ax_swept_path.plot(self.line_length*np.array((0,np.cos(self.max_path_cone_rad))), self.line_length*np.array((0,np.sin(self.max_path_cone_rad))), color='k')
		self.ax_swept_path.plot(self.line_length*np.array((0,np.cos(self.min_path_cone_rad))), self.line_length*np.array((0,np.sin(self.min_path_cone_rad))), color='k')
		self.ax_swept_path.set_xlim((-1,3))
		self.ax_swept_path.set_ylim((-3,3))
		
		plt.show(block=False)
		plt.pause(0.00001)


if __name__=="__main__":
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	path_calculator=PathCalculator()
	target_theta_deg=0.0
	target_speed=path_calculator.v
	try:
		print msg
		while True:
			key = getKey()
			if key == 'd' :
			    target_theta_deg = checkPathLimitAngle(target_theta_deg - THETA_DEG_INC)
			    print pathMsg(target_theta_deg, target_speed)
			if key == 'a' :
			    target_theta_deg = checkPathLimitAngle(target_theta_deg + THETA_DEG_INC)
			    print pathMsg(target_theta_deg, target_speed)
			if key == 'w' :
			    target_speed = checkPathLimitSpeed(target_speed + SPEED_INC)
			    print pathMsg(target_theta_deg, target_speed)
			if key == 's' :
			    target_speed = checkPathLimitSpeed(target_speed - SPEED_INC)
			    print pathMsg(target_theta_deg, target_speed)
			else:
				if (key == '\x03'):
					break

			if (key!=''):
				path_calculator.v=target_speed
				path_calculator.plot_car_line(target_theta_deg*np.pi/180)

	# except:
	# 	print e

	finally:
		pass

	if os.name != 'nt':
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)