#! /usr/bin/env python

# Python script to cluster raw 2D lidar data into lines and blobs (tested with turtlebot3_gazebo)

import rospy
import numpy as np

import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from sklearn import linear_model
from scipy import interpolate

from matplotlib import cm
import matplotlib.patheffects as pe
from matplotlib.patches import Circle 

from sensor_msgs.msg import LaserScan

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

class obstacle_cluster:
	def __init__(self):

		# Initialize Node
		rospy.init_node('turtlebot3', anonymous=True)

		# Initialize Laser Scan Subscriber
		sub = rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)

		# Initialize Data
		self.laser_scan=np.empty(360)
		self.angle_scan=np.linspace(0,2*np.pi,len(self.laser_scan))

		# Initialize Scaler, DBSCAN and RANSAC
		self.standard_scaler=StandardScaler()
		self.dbscan=DBSCAN(eps=0.2, min_samples=3)
		self.dbscan_line=DBSCAN(eps=0.1, min_samples=5)
		self.ransac=linear_model.RANSACRegressor(residual_threshold=0.1)

		# Initialize Lidar Plot
		self.fig = plt.figure()
		self.ax_lidar= self.fig.add_subplot(1,2,1)
		plt.gca().set_aspect('equal', adjustable='box')
		self.ax_lidar.set_xlim((-2,2))
		self.ax_lidar.set_ylim((-2,2))

		# Initialize Object Plot
		self.ax_object= self.fig.add_subplot(1,2,2)
		plt.gca().set_aspect('equal', adjustable='box')
		self.ax_object.set_xlim((-2,2))
		self.ax_object.set_ylim((-2,2))
		

		

		# Initialize Rate
		self.rate=rospy.Rate(10)

		# Initialize parameters
		self.line_threshold=0.4
		self.acc_threshold=0.25
		self.n_clusters_=0
		self.n_noise_=0

		# Initialize shapes
		self.lines_found=np.empty((0,4))
		self.blobs_found=np.empty((0,3))


	def laser_scan_callback(self, msg):
		self.laser_scan=np.asarray(msg.ranges)

	def find_obstacles(self):

		# While not shutdown
		while not rospy.is_shutdown():

			# Read
			rospy.wait_for_message('/scan', LaserScan)

			# Transform and Scale
			self.transform_laser()

			# Apply DBSCAN
			db = self.dbscan.fit(self.xy_laser)

			self.core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
			self.core_samples_mask[db.core_sample_indices_] = True
			self.labels = db.labels_
			self.n_clusters_ = len(set(self.labels)) - (1 if -1 in self.labels else 0)
			self.n_noise_ = list(self.labels).count(-1)

			# Plot
			self.update_plot()
			self.find_objects()
			self.rate.sleep()
			self.plot_objects()

	def transform_laser(self):
		valid_laser=(self.laser_scan!=np.inf)
		x_laser=self.laser_scan[valid_laser]*np.cos(self.angle_scan[valid_laser])
		y_laser=self.laser_scan[valid_laser]*np.sin(self.angle_scan[valid_laser])
		self.xy_laser=np.array((x_laser, y_laser)).T
		#self.xy_laser = self.standard_scaler.fit_transform(self.xy_laser) 


	def update_plot(self):
		# Plot result		
		self.ax_lidar.clear()

		
		# Black removed and is used for noise instead.
		unique_labels = set(self.labels)
		colors = [plt.cm.Spectral(each)
		          for each in np.linspace(0, 1, len(unique_labels))]
		for k, col in zip(unique_labels, colors):
		    if k == -1:
		        # Black used for noise.
		        col = [0, 0, 0, 1]

		    class_member_mask = (self.labels == k)

		    xy = self.xy_laser[class_member_mask & self.core_samples_mask]
		    self.ax_lidar.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
		             markeredgecolor='k', markersize=14)

		    xy = self.xy_laser[class_member_mask & ~self.core_samples_mask]
		    self.ax_lidar.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
		             markeredgecolor='k', markersize=6)
		    
		    self.ax_lidar.arrow(0,0,0.25,0, length_includes_head=True, shape="full", head_width=0.05)

		self.ax_lidar.set_title('Estimated number of clusters: %d' % self.n_clusters_)
		# Set Lim
		self.ax_lidar.set_xlim((-2,2))
		self.ax_lidar.set_ylim((-2,2))

		plt.show(block=False)
		plt.pause(0.0001)

	
	def find_objects(self):
		
		unique_labels = set(self.labels)
		  
		for k in unique_labels:
		    if k == -1:
		        # Noise
		        pass
		    else:
		    	class_member_mask = (self.labels == k)
		    	xy = self.xy_laser[class_member_mask & self.core_samples_mask]
		    	if not xy.size==0:
		    		if self.is_line(xy):
		    			self.make_lines(xy)
		    		else:
		    			self.make_blob(xy)

	def plot_objects(self):

		self.ax_object.clear()

		self.ax_object.arrow(0,0,0.25,0, length_includes_head=True, shape="full", head_width=0.05)
		
		for row in self.lines_found:
			self.ax_object.plot(row[0:2], row[2:4], c='k', linewidth=2, path_effects=[pe.Stroke(linewidth=3, foreground='k'), pe.Normal()])
		
		for row in self.blobs_found:
			self.ax_object.add_artist(Circle(xy=(row[0], row[1]), radius=row[2], color='k'))

		self.ax_object.set_title('Estimated number of walls: {number_of_walls}\nEstimated number of obstacles: {number_of_blobs}'.format(number_of_walls=self.lines_found.shape[0], number_of_blobs=self.blobs_found.shape[0]))
		
		# Set Lim
		self.ax_object.set_xlim((-2,2))
		self.ax_object.set_ylim((-2,2))

		plt.show(block=False)
		plt.pause(0.0001)
	
		# Clear objects
		self.lines_found=np.empty((0,4))
		self.blobs_found=np.empty((0,3))





	def hough_transform(self, xy):
		# Find centroid
		centroid=np.sum(xy, axis=0)/xy.shape[0]
		# Center data
		xy_origin=xy-centroid
		# Compute differences
		xy_origin_gradient=np.gradient(xy_origin, axis=0)
		xy_origin_gradient=self.gradient(xy)
		# Compute normals
		v=xy_origin_gradient[:,[1,0]]
		v[:,0]=-v[:,0]
		# Compute norm
		v_norm=np.sqrt(np.square(v[:,0])+np.square(v[:,1]))
		# Compute distance
		distance=(xy[:,0]*v[:,0]+xy[:,1]*v[:,1])/v_norm
		# Compute angle
		angle=np.arctan2(v[:,1], v[:,0])
		# 0 angles lines are rare so this is where we cut
		angle[angle<0]=angle[angle<0]+2*np.pi

		return np.array((distance, angle)).T

	def is_line(self, xy):
		# Find centroid
		centroid=np.sum(xy, axis=0)/xy.shape[0]
		# Center data
		xy_origin=xy-centroid
		# Compute moment of inertia
		moment=np.sum(np.sqrt(np.square(xy_origin[:,0])+np.square(xy_origin[:,1])), axis=0)
		# Divide by length?
		return moment/xy.shape[0]>self.line_threshold # Variable parameter

	def make_lines(self, xy):
		
		hough=self.hough_transform(xy)
		db_line = self.dbscan_line.fit(hough)

		core_samples_mask_line = np.zeros_like(db_line.labels_, dtype=bool)
		core_samples_mask_line[db_line.core_sample_indices_] = True
		labels_line = db_line.labels_
	
		

		# Black removed and is used for noise instead.
		unique_labels = set(labels_line)
		colors = [plt.cm.Spectral(each) # To be removed
		          for each in np.linspace(0, 1, len(unique_labels))]
		for k, col in zip(unique_labels, colors):
		    if k == -1:
		        # Black used for noise.
		        col = [0, 0, 0, 1]

		    class_member_mask = (labels_line == k)

		    # line = xy[class_member_mask & core_samples_mask_line]
		    # ax2.plot(line[:, 0], line[:, 1], 'o', markerfacecolor=tuple(col), markeredgecolor='k', markersize=14)
		    # line = hough[class_member_mask & core_samples_mask_line]
		    # ax3.plot(line[:, 0], line[:, 1], 'o', markerfacecolor=tuple(col), markeredgecolor='k', markersize=14)

		    # line = xy[class_member_mask & ~core_samples_mask_line]
		    # ax2.plot(line[:, 0], line[:, 1], 'o', markerfacecolor=tuple(col), markeredgecolor='k', markersize=6)
		    # line = hough[class_member_mask & ~core_samples_mask_line]
		    # ax3.plot(line[:, 0], line[:, 1], 'o', markerfacecolor=tuple(col), markeredgecolor='k', markersize=14)
		    
		    # RANSAC
		    if k == -1:
		    	break
		    line = xy[class_member_mask]
		    
		    self.ransac.fit(line[:, 0].reshape((-1,1)), line[:, 1])
		    
		    inlier_mask = self.ransac.inlier_mask_
		    outlier_mask = np.logical_not(inlier_mask) # Can be removed

		    line=line[inlier_mask]

		    line_X = np.array((line[:,0].min(), line[:,0].max())).reshape((-1,1))
		    line_y_ransac = self.ransac.predict(line_X)
		    self.lines_found=np.append(self.lines_found, np.concatenate((line_X.reshape((-1,2)), line_y_ransac.reshape((-1,2))),axis=1), axis=0)
		    
		    ###ax2.plot(line_X, line_y_ransac, color=tuple(col), linewidth=2, path_effects=[pe.Stroke(linewidth=3, foreground='k'), pe.Normal()])
		    

		####plt.show(block=False)

	def make_blob(self, xy):
		# Find centroid
		centroid=np.sum(xy, axis=0)/xy.shape[0]
		# Center data
		xy_origin=xy-centroid
		# Compute radius

		radius=np.sum(np.sqrt(np.square(xy_origin[:,0])+np.square(xy_origin[:,1])), axis=0)/xy.shape[0]
		
		self.blobs_found=np.append(self.blobs_found, np.concatenate((centroid,np.array([radius]))).reshape(-1,3), axis=0)
		    

	def gradient(self, xy):

		# Compute prelimary diff
		diff4=np.roll(xy,-3, axis=0)-np.roll(xy,3, axis=0)
	
		# Compute first 4
		diff4[0,:]=xy[4,:]-xy[0,:]
		diff4[1,:]=xy[5,:]-xy[1,:]
		diff4[2,:]=xy[6,:]-xy[2,:]
		diff4[3,:]=xy[7,:]-xy[3,:]
		# Compute last 4
		diff4[-1,:]=xy[-1,:]-xy[-5,:]
		diff4[-2,:]=xy[-2,:]-xy[-6,:]
		diff4[-3,:]=xy[-3,:]-xy[-7,:]
		diff4[-4,:]=xy[-4,:]-xy[-8,:]
		
		###print(diff4)

		# Normalize
		diff4_norm=np.sqrt(np.square(diff4[:,0])+np.square(diff4[:,1]))[:, None]
		diff4_normalized=diff4/diff4_norm
		# Compute diff acceleration
		acc4=np.roll(diff4_normalized,-3, axis=0)-np.roll(diff4_normalized,3, axis=0)
		
		acc4_norm=np.sqrt(np.square(acc4[:,0])+np.square(acc4[:,1]))[:, None]

		# # Replace values with acc4 higher than self.acc_threshold with nearest value
		# fig_acc = plt.figure()
		# ax_acc= fig_acc.add_subplot(1,1,1)
		# ax_acc.plot(np.array((range(acc4.shape[0]))),acc4_norm)
		# ax_acc.plot(np.array((range(acc4.shape[0]))),diff4[:,0])

		mask = (acc4_norm>self.acc_threshold)
		idx=np.array((range(diff4.shape[0]))).reshape((-1,1))
		
		x_diff4=diff4[:,0].reshape((-1,1))
		y_diff4=diff4[:,1].reshape((-1,1))

		# print(x_diff4.shape)
		# print(mask.shape)
		# print(x_diff4[~mask])
		
		interpolator_x=interpolate.interp1d(idx[~mask],x_diff4[~mask], kind='nearest', fill_value='extrapolate')
		interpolator_y=interpolate.interp1d(idx[~mask],y_diff4[~mask], kind='nearest', fill_value='extrapolate')
		
		x_diff4[mask]=interpolator_x(idx[mask])
		y_diff4[mask]=interpolator_y(idx[mask])


		test=np.concatenate((x_diff4, y_diff4),axis=1)
		
		###ax_acc.plot(np.array((range(acc4.shape[0]))),x_diff4)
		
		###plt.show(block=False)
		

		return test




if __name__=='__main__':
	try:
		obstacle_cluster_object=obstacle_cluster()
		obstacle_cluster_object.find_obstacles()
	except rospy.ROSInterruptException:
		pass
