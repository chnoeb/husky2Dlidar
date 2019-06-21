#! /usr/bin/env python


# Python script for artificial potential field algorithm

import numpy as np
from math import cos, sin, atan2
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
from matplotlib import cm
import time


# THINGS TO FIX: -1 to 1 for grid, 50 for max
FIELD_CELLING=10
MIN_GRID=-1.0
MAX_GRID=1.0
GRID_INC=0.02

def read_laser_scan():
	fid=open("laser_scan_sample.txt", 'r')
	if fid.mode=='r':
		return fid.read()
		
def transform_laser_scan(laser_scan):
	laser_scan=laser_scan.split(',')
	laser_scan=[value.strip() for value in laser_scan]
	laser_scan=[value.replace('inf','10') for value in laser_scan]
	laser_scan=[float(value) for value in laser_scan]
	return np.asarray(laser_scan)


def compute_x_y(laser_scan,angle_scan):
	
	x_laser=laser_scan*np.cos(angle_scan)
	y_laser=laser_scan*np.sin(angle_scan)

	return x_laser, y_laser

def compute_rho(x,y,x_laser,y_laser):
	# Implement brushfire algorithm
	return min(np.sqrt((x-x_laser)**2+(y-y_laser)**2))

def create_grid():
	x_grid=np.arange(MIN_GRID, MAX_GRID+GRID_INC, GRID_INC)
	y_grid=np.arange(MIN_GRID, MAX_GRID+GRID_INC, GRID_INC)
	return np.meshgrid(x_grid, y_grid, sparse=False, indexing='ij')

def brushfire(x_grid, y_grid, x_laser, y_laser):
	
	# Delete outliers
	laser_inliers=(x_laser>=MIN_GRID)&(x_laser<=MAX_GRID)&(y_laser>=MIN_GRID)&(y_laser<=MAX_GRID)
	x_laser=x_laser[laser_inliers]
	y_laser=y_laser[laser_inliers]

	# Inverse interpolation
	x_laser_idx=(GRID_INC*np.round(x_laser/GRID_INC)+MAX_GRID)/GRID_INC
	x_laser_idx=x_laser_idx.astype(int)
	y_laser_idx=(GRID_INC*np.round(y_laser/GRID_INC)+MAX_GRID)/GRID_INC
	y_laser_idx=y_laser_idx.astype(int)


	# Initial distance
	dist=1
	# Add border to iterate on neighbors
	rho_field=np.zeros((x_grid.shape[0]+2,x_grid.shape[1]+2))
	# Account for border
	rho_field[x_laser_idx+1,y_laser_idx+1]=dist

	# Brushfire algorithm
	x_neighbors=np.array([1,1,0,-1,-1,-1,0,1])
	y_neighbors=np.array([0,1,1,1,0,-1,-1,-1])

	while np.any(rho_field==0):
		#Account for border
		x_frontier, y_frontier=np.where(rho_field[1:-1,1:-1]==dist)
		x_frontier+=1
		y_frontier+=1
		dist+=1

		for i in range(len(x_neighbors)):
			x_neighbor_frontier=x_frontier+x_neighbors[i]
			y_neighbor_frontier=y_frontier+y_neighbors[i]

			# Eliminate already done points
			valid_neighbors=rho_field[x_neighbor_frontier,y_neighbor_frontier]==0
			x_neighbor_frontier=x_neighbor_frontier[valid_neighbors]
			y_neighbor_frontier=y_neighbor_frontier[valid_neighbors]

			# Propagate new distance
			rho_field[x_neighbor_frontier,y_neighbor_frontier]=dist
	
	# Remove border
	rho_field=rho_field[1:-1,1:-1]
	# Scale accordingly
	rho_field*=GRID_INC

	return rho_field



def create_repulsive_field(x_grid, y_grid, x_laser, y_laser):
	d0=0.4
	eta=0.01

	ny, nx = x_grid.shape

	repulsive_field=np.zeros(x_grid.shape)

	# Old way of computing rho_field (slower)

	#rho_field=np.empty(x_grid.shape)

	# for i in range(nx):
	# 	for j in range (ny):
	# 		rho_field[i,j]=compute_rho(x_grid[i,j], y_grid[i,j], x_laser, y_laser)
	# 		if rho_field[i,j]<=d0:
	# 			repulsive_field[i,j]=eta*((1/rho_field[i,j])-(1/d0))**2
	# 			# For visualization
	# 			if repulsive_field[i,j]>FIELD_CELLING:
	# 				repulsive_field[i,j]=FIELD_CELLING
	# 		else:
	# 			repulsive_field[i,j]=0

	rho_field=brushfire(x_grid, y_grid, x_laser, y_laser)
	repulsive_field[rho_field<d0]=eta*((1/rho_field[rho_field<d0])-(1/d0))**2
	
	# Add max here if needed


	return repulsive_field


def create_attactive_field(x_grid, y_grid, x_goal=1.0, y_goal=0):
	
	xi=2.0
	attractive_field=xi*((x_goal-x_grid)**2+(y_goal-y_grid)**2)
	return attractive_field

def compute_gradients(potential_field, x_grid, y_grid):
	x_gradient, y_gradient=np.gradient(potential_field, x_grid[:,0], y_grid[0,:])
	return x_gradient, y_gradient

def find_gradient_direction(x_gradient, y_gradient):
	# Find grid robot
	robot_grid_idx=int((-MIN_GRID/(MAX_GRID-MIN_GRID))*(MAX_GRID-MIN_GRID)/GRID_INC)
	
	return atan2(-y_gradient[robot_grid_idx][robot_grid_idx], -x_gradient[robot_grid_idx][robot_grid_idx])


def plot_fields(x_grid, y_grid, x_laser, y_laser, repulsive_field, attractive_field, potential_field):
	fig=plt.figure()

 	ax=fig.add_subplot(221,projection='3d')
 	ax.plot_surface(x_grid, y_grid, repulsive_field, cmap=cm.coolwarm, vmin=0, vmax=10)
 	ax.set_xlabel('x')
 	ax.set_ylabel('y')
 	ax.set_zlabel('Repulsive Field')

 	
 	ax.set_xlim3d(MIN_GRID,MAX_GRID)
 	ax.set_ylim3d(MIN_GRID,MAX_GRID)
 	ax.set_zlim3d(0,FIELD_CELLING)
 	ax.view_init(azim=-45,elev=35)
 	plt.gca().set_aspect('equal', adjustable='box')


 	ax=fig.add_subplot(222,projection='3d')
 	ax.plot_surface(x_grid, y_grid, attractive_field, cmap=cm.coolwarm, vmin=0, vmax=10)
 	ax.set_xlabel('x')
 	ax.set_ylabel('y')
 	ax.set_zlabel('Attractive Field')

 	
 	ax.set_xlim3d(MIN_GRID,MAX_GRID)
 	ax.set_ylim3d(MIN_GRID,MAX_GRID)
 	ax.set_zlim3d(0,FIELD_CELLING)
 	ax.view_init(azim=-45,elev=35)
 	plt.gca().set_aspect('equal', adjustable='box')



 	ax=fig.add_subplot(223,projection='3d')
 	ax.plot_surface(x_grid, y_grid, potential_field, cmap=cm.coolwarm, vmin=0, vmax=10)
 	ax.set_xlabel('x')
 	ax.set_ylabel('y')
 	ax.set_zlabel('Potential Field')

 	
 	ax.set_xlim3d(MIN_GRID,MAX_GRID)
 	ax.set_ylim3d(MIN_GRID,MAX_GRID)
 	ax.set_zlim3d(0,FIELD_CELLING)
 	ax.view_init(azim=-45,elev=35)
 	plt.gca().set_aspect('equal', adjustable='box')


 	ax=fig.add_subplot(224)
 	ax.scatter(x_laser,y_laser)
 	ax.plot(0,0,'rx')
 	
  	ax.set_xlabel('x')
 	ax.set_ylabel('y')
 	ax.set_xlim(MIN_GRID,MAX_GRID)
 	ax.set_ylim(MIN_GRID,MAX_GRID)
 	plt.gca().set_aspect('equal', adjustable='box')

 	plt.show()

def compute_gradient_direction(laser_scan):

	laser_scan[laser_scan==np.inf]=10.0
	angle_scan=np.linspace(0,2*np.pi,len(laser_scan))
	x_laser, y_laser=compute_x_y(laser_scan,angle_scan)
	x_grid, y_grid=create_grid()

	repulsive_field=create_repulsive_field(x_grid, y_grid, x_laser, y_laser)
	attractive_field=create_attactive_field(x_grid, y_grid)
 	potential_field=attractive_field+repulsive_field

 	x_gradient, y_gradient=compute_gradients(potential_field, x_grid, y_grid)
 	
 	#plot_fields(x_grid, y_grid, x_laser, y_laser, repulsive_field, attractive_field, potential_field)
 	
 	return find_gradient_direction(x_gradient, y_gradient)


if __name__=="__main__":
 	laser_scan=read_laser_scan()
 	laser_scan=transform_laser_scan(laser_scan)
 	laser_scan=np.roll(laser_scan,0)
 	angle_scan=np.linspace(0,2*np.pi,len(laser_scan))

 	x_laser, y_laser=compute_x_y(laser_scan,angle_scan)

 	x_grid, y_grid=create_grid()

 	repulsive_field=create_repulsive_field(x_grid, y_grid, x_laser, y_laser)
	attractive_field=create_attactive_field(x_grid, y_grid)
 	potential_field=attractive_field+repulsive_field

 	x_gradient, y_gradient=compute_gradients(potential_field, x_grid, y_grid)

 	gradient_direction=find_gradient_direction(x_gradient, y_gradient)
 	print(gradient_direction)

 	plot_fields(x_grid, y_grid, x_laser, y_laser, repulsive_field, attractive_field, potential_field)

 	


 	


 	