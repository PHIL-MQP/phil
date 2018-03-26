#!/usr/bin/python3

# x -> 0.06317
# y -> 0.02687

import numpy as np
import sys
import matplotlib.pyplot as plt
import csv
import matplotlib.cm as cm
from matplotlib.patches import Ellipse, Circle
import struct
import argparse

np.set_printoptions(suppress=True)  # no scientific notation

parser = argparse.ArgumentParser("Compute error of aruco estimate pose")
parser.add_argument("detectmarkersfile", help="The detect markers output")
parser.add_argument("mocaptagsfile", help="The hand-written tag poses from vicon")
parser.add_argument("mocaprobotfile", help="The robot poses from vicon")
parser.add_argument("timestampsfile", help="The timestamps file for video stream")
parser.add_argument("tagpointsfile", help="Tag points csv")

args = parser.parse_args()

# Read in robot pose from mocap data
def parse_bot_pose(filename):
	data =  np.genfromtxt(filename, delimiter=',')
	data[:,0] -= data[0,0]
	data[:,0] *= 0.01 # convert timestamps to seconds
	# print(data[:,0])
	return data

def get_camera_start(filename):
	data =  np.genfromtxt(filename)
	# data[:] -= data[0]
	data[:] *= 0.000001 # convert timestamps to seconds
	# print(data[0])
	return data[0]

def parse_aruco_data(arucofile, timestampsfile):
	data =  np.genfromtxt(arucofile, delimiter=',')
	ts =  np.genfromtxt(timestampsfile)
	data[:,0] -= ts[0]
	data[:,0] *= 0.000001 # convert timestamps to seconds
	
	# print(data)
	return data

def parse_tag_poses(tagfile):
	data = np.genfromtxt(tagfile, delimiter=',')

	# print(data)
	return data

def compute_error(aruco, robot, tag):

	errors = []
	bad_pose = 0

	for i in range(aruco.shape[0]):

		# loop through known tag IDs and find matches
		for j in range(tag.shape[0]):
			if( tag[j,0] == aruco[i,1] ):
				if(aruco[i,2] == -999999.):
					bad_pose += 1
				else:
					# find the robot pose that matches this timestamp
					tsIdxT = (np.abs(robot[:,0]-aruco[i,0])).argmin()

					mocap_tf = np.array([robot[tsIdxT,1] - tag[j,1], # x
											robot[tsIdxT,2] - tag[j,2], # y 
											robot[tsIdxT,3] - tag[j,3] ]) # z

					mocap_tf *= 0.001
					# print(mocap_tf)
					# print(robot[tsIdxT,:])
					# print(aruco[i,2:5])
					# print()
					error = np.abs(mocap_tf - np.array([aruco[i,4], aruco[i,2], aruco[i,3]]) )
					error = np.array([aruco[i,0], aruco[i,1], error[0], error[1], error[2]])
					errors.append(error)
				break
	errors = np.array(errors)
	# for e in errors:
	# 	print(e)
	print("Bad Poses", str(bad_pose))
	print("Total Detected", str(aruco.shape[0]))
	print("Dropped Pct", str(bad_pose*100./float(aruco.shape[0])))
	return errors

def hex2rgb(rgb):
    temp =  struct.unpack('BBB', rgb.decode('hex'))
    return [temp[0]/255.,temp[1]/255.,temp[2]/255.,1.]

def plotPoses():
	robot_pose = parse_bot_pose(args.mocaprobotfile)
	tag_poses = parse_tag_poses(args.mocaptagsfile)
	aruco_data = parse_aruco_data(args.detectmarkersfile, args.timestampsfile)

	errors = compute_error(aruco_data, robot_pose, tag_poses)

	# mean_x = np.mean(errors[2,:])
	# med_x = np.median(errors[2,:])
	# pct95_x = np.percentile(errors[2:], 95)
	# pct5_x = np.percentile(errors[2:], 5)

	# print(mean_x, med_x, pct95_x, pct5_x)
	plt.figure(figsize=(15,15), facecolor='black')

	# 				0,				4,				8,				14,				16,				20,				24,				30				32				36  			40 				44
	# 				red, 			orange,		yellow,		l green,	d green, 	l blue 	, d blue 	, purple		pink 			grey			brown			black
	# colors = ['CD5555', 'FF6103', 'E3CF57', '00FF00', '6E8B3D', '00BFFF', '0000FF', '9932CC', 'FF69B4', '8B8386', '9C661F', '000000' ]
	colors = cm.rainbow(np.linspace(0, 1, tag_poses.shape[0]))

	c_data = []
	for i in range(errors.shape[0]):
		idx =  np.where(tag_poses[:,0] == int(errors[i,1]) )
		c_data.append( colors[idx[0][0]] )
		# c_data.append( hex2rgb( colors[idx[0][0]] ))

	# plt.scatter(errors[:,2], errors[:,3], s=10, color=c_data)
	# plt.ylabel("Error Y (meters)", color='white')
	# plt.xlabel("Error X (meters)", color='white')
	# plt.title("X Y Error from all Tags (position)", color='white')
	# plt.tick_params(colors='white')
	# plt.axis("equal")
	# plt.show()

	# robot_pose[:,1] -= tag_poses[0,1]
	# plt.scatter(robot_pose[:,0], robot_pose[:,3]*0.001, s=10, color=colors)
	# plt.ylabel("Z (meters)", color='white')
	# plt.xlabel("time (seconds)", color='white')
	# plt.title("Z Mocap (position)", color='white')
	# plt.tick_params(colors='white')
	# plt.show()

	i = 0
	temp = []
	while(i < aruco_data.shape[0]):
		if( aruco_data[i,2] < -999998.):
			pass
		else:
			temp.append(aruco_data[i,:])
		i += 1

	aruco_data = np.array(temp)
	temp = []
	for i in range(aruco_data.shape[0]):
		idx =  np.where(tag_poses[1,0] == int(aruco_data[i,1]) )
		if(idx[0].shape[0] == 0):
			pass
		else:
			temp.append( aruco_data[i,:] )
			c_data.append( colors[idx[0][0]] )

	aruco_data = np.array(temp)

	pose_from_tag8  = []

	robot_pose[:,1] -= tag_poses[2,1]
	robot_pose[:,2] -= tag_poses[2,2]
	robot_pose[:,3] -= tag_poses[2,3]

	robot_pose[:,1] *= 0.001
	robot_pose[:,2] *= 0.001
	robot_pose[:,3] *= 0.001

	# time, id, x, y, z, rots...
	# time, id, z, 
	print(aruco_data[0,:])
	print(aruco_data[-1,:])
	print('\n')

	# time, x, y, z, rots...
	print(robot_pose[0,:])
	print(robot_pose[-1,:])
	print('\n')

	# id, x, y, z, ...
	print(tag_poses[0,:])

	print(aruco_data[-1,0] - aruco_data[0,0])
	print(robot_pose[-1,0] - robot_pose[0,0])

	# print(aruco_data)
	# temp = []
	# for i in range((aruco_data.shape[0])):
	# 	if(aruco_data[i,1] == 0):
	# 		temp.append(aruco_data[i,:])
	# aruco_data = np.array(temp)

	colors = cm.rainbow(np.linspace(0, 1, aruco_data.shape[0]))

	plt.scatter(aruco_data[:,0], aruco_data[:,4], s=10, color='g')
	plt.scatter(robot_pose[:,0], robot_pose[:,2], s=10, color='r')

	# plt.scatter(aruco_data[:,0], aruco_data[:,3], s=10, color='g')
	# plt.scatter(robot_pose[:,0], robot_pose[:,3], s=10, color='r')

	# plt.scatter(aruco_data[:,0], aruco_data[:,2], s=10, color='g')
	# plt.scatter(robot_pose[:,0], robot_pose[:,1], s=10, color='r')
	# plt.scatter(aruco_data[:,4], aruco_data[:,2], s=10, color=colors)


	plt.title("Y distance from Tag 0 (aruco vs mocap) over time", color='white')
	plt.ylabel("Robot Y (meters)", color='white')
	plt.xlabel("time (seconds)", color='white')
	plt.tick_params(colors='white')

	plt.show()

# return x y z numpy array
def computeRotation(p1, p2, p3):
	v1 = np.array([p2[i] - p1[i] for i in range(3) ])
	v2 = np.array([p3[i] - p1[i] for i in range(3) ])
	v1_x_v2 = np.cross(v1,v2)

	# radius of vector
	r = np.sqrt( v1_x_v2[0]**2 + v1_x_v2[1]**2 + v1_x_v2[2]**2 )
	# angle in x-y plane
	t = np.arctan2(v1_x_v2[1], v1_x_v2[0])
	# angle away from z axis
	p = np.arccos(v1_x_v2[2]/r)

	v3 = np.cross(v1_x_v2,v2) 

	print("Vect3",v3)

	print([r, t,  p])

# return a list of 3 global points
# top left, right, bottom left
def applyOffsets(centroid, off1, off2, off3):
	
	temp1 = [(centroid[0] + off1[0]), (centroid[1] + off1[1]), (centroid[2] + off1[2])]
	temp2 = [(centroid[0] + off2[0]), (centroid[1] + off2[1]), (centroid[2] + off2[2])]
	temp3 = [(centroid[0] + off3[0]), (centroid[1] + off3[1]), (centroid[2] + off3[2])]
	temp1 = temp1 + temp2 + temp3
	ptArray = np.array( temp1 )
	return ptArray

def getPointPoses(centroids, marker_poses):

	if(centroids.shape[0] != marker_poses.shape[0]):
		print("Error: matrix shape mismatch")
		return -2

	temp = np.array(applyOffsets(centroids[0,1:4], marker_poses[0,1:4],  marker_poses[0,3:6],  marker_poses[0,5:9] ))
	for i in range(1,centroids.shape[0]):
		if(centroids[i,0] == marker_poses[i,0]):
			temp = np.vstack((temp, applyOffsets(centroids[i,1:4], marker_poses[i,1:4],  marker_poses[i,3:6],  marker_poses[i,5:9] )) )
		else:
			print("Mismatching IDS ", str(centroids[i,0]), " cent and ", str(marker_poses[i,0])," offsets" )

	print(temp)
	return temp

def parseTagPoints(pointsFile):
	data =  np.genfromtxt(pointsFile, delimiter=',',skip_header=1)
	return data


def main():
	robot_pose = parse_bot_pose(args.mocaprobotfile)
	tag_poses = parse_tag_poses(args.mocaptagsfile)
	aruco_data = parse_aruco_data(args.detectmarkersfile, args.timestampsfile)

	errors = compute_error(aruco_data, robot_pose, tag_poses)

	tag_points = parseTagPoints(args.tagpointsfile)

	# remove any bad detections
	i = 0
	temp = []
	while(i < aruco_data.shape[0]):
		if( aruco_data[i,2] < -999998.):
			pass
		else:
			temp.append(aruco_data[i,:])
		i += 1
	aruco_data = np.array(temp)

	colors = cm.rainbow(np.linspace(0, 1, tag_poses.shape[0]))

	# get only data for tag ID 4
	temp = []
	c_data = []
	for i in range(aruco_data.shape[0]):
		idx =  np.where(tag_poses[1,0] == int(aruco_data[i,1]) )
		if(idx[0].shape[0] == 0):
			pass
		else:
			temp.append( aruco_data[i,:] )
			c_data.append( colors[idx[0][0]] )

	aruco_data = np.array(temp)

	robot_pose[:,1] -= tag_poses[2,1]
	robot_pose[:,2] -= tag_poses[2,2]
	robot_pose[:,3] -= tag_poses[2,3]

	robot_pose[:,1] *= 0.001
	robot_pose[:,2] *= 0.001
	robot_pose[:,3] *= 0.001

	# time, id, x, y, z, rots...
	# time, id, z, 
	# print(aruco_data[0,:])
	# print(aruco_data[-1,:])
	# print('\n')

	# time, x, y, z, rots...
	# print(robot_pose[0,:])
	# print(robot_pose[-1,:])
	# print('\n')

	# id, x, y, z, ...
	# print(tag_poses[1,:])

	# print(aruco_data[-1,0] - aruco_data[0,0])
	# print(robot_pose[-1,0] - robot_pose[0,0])

	plt.figure(figsize=(15,15), facecolor='black')

	computeRotation([1,0,0], [0,1,0], [1,1,0])

	points = getPointPoses(tag_poses, tag_points)

	print(points[0,0:3].shape)
	print(points[0,3:6].shape)
	print(points[0,5:8].shape)

	print(computeRotation(points[0,0:3], points[0,3:6], points[0,5:8]))

	print(points.shape)

	colors = cm.rainbow(np.linspace(0, 1, aruco_data.shape[0]))

	# top left
	plt.scatter(points[:,0], points[:,1], s=10, color='r') 
	# right
	plt.scatter(points[:,3], points[:,4], s=10, color='g')
	# bottom left
	plt.scatter(points[:,6], points[:,7], s=10, color='b')

	# plt.scatter(robot_pose[:,0], robot_pose[:,2], s=10, color='r')

	# plt.scatter(aruco_data[:,0], aruco_data[:,4], s=10, color='g')
	# plt.scatter(robot_pose[:,0], robot_pose[:,2], s=10, color='r')

	# plt.scatter(aruco_data[:,0], aruco_data[:,3], s=10, color='g')
	# plt.scatter(robot_pose[:,0], robot_pose[:,3], s=10, color='r')

	# plt.scatter(aruco_data[:,0], aruco_data[:,2], s=10, color='g')
	# plt.scatter(robot_pose[:,0], robot_pose[:,1], s=10, color='r')
	# plt.scatter(aruco_data[:,4], aruco_data[:,2], s=10, color=colors)


	# plt.title("Y distance from Tag 0 (aruco vs mocap) over time", color='white')
	# plt.ylabel("Robot Y (meters)", color='white')
	# plt.xlabel("time (seconds)", color='white')
	plt.tick_params(colors='white')

	plt.show()

if __name__ == '__main__':
    sys.exit(main())
