#!/usr/bin/python3

# x -> 0.06317
# y -> 0.02687

import numpy as np
import sys
import matplotlib.pyplot as plt
import csv
import matplotlib.cm as cm
from matplotlib.patches import Ellipse, Circle
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import struct
import argparse
import math

np.set_printoptions(suppress=True)  # no scientific notation

parser = argparse.ArgumentParser("Compute error of aruco estimate pose")
parser.add_argument("detectmarkersfile", help="The detect markers output")
parser.add_argument("mocaptagsfile", help="The hand-written tag poses from vicon")
parser.add_argument("mocaprobotfile", help="The robot poses from vicon")
parser.add_argument("timestampsfile", help="The timestamps file for video stream")
parser.add_argument("tagpointsfile", help="Tag points csv")

args = parser.parse_args()

# https://stackoverflow.com/a/31364297/3353601
def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.
    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

# Read in robot pose from mocap data
# convert timestamps (ts) to seconds
# convert pose data to meters
# output as ts, tx, ty, tx, rx, ry, rz
def parse_bot_pose(filename):
	data =  np.genfromtxt(filename, delimiter=',')
	data[:,0] -= data[0,0]
	data[:,0] *= 0.01 # convert timestamps to seconds
	data[:,1:] *= 0.001 #convert pose data to meters

	return data

# get the first timestamp from the camera stream in seconds
def get_camera_start(filename):
	data =  np.genfromtxt(filename)
	# data[:] -= data[0]
	data[:] *= 0.000001 # convert timestamps to seconds

	return data[0]

# return the aruco data in the format below:
# ts, tx, ty, tx, rx, ry, rz
# rototations are representated as a 1x3 rotational vector
def parse_aruco_data(arucofile, timestampsfile):
	data =  np.genfromtxt(arucofile, delimiter=',')
	ts =  np.genfromtxt(timestampsfile)
	data[:,0] -= ts[0]
	data[:,0] *= 0.000001 # convert timestamps to seconds

	# print(data)
	return data, ts

# return the centroids of tags as coordinates from abs zero of the mocap system
def parse_tag_centroids(tagfile):
	data = np.genfromtxt(tagfile, delimiter=',',skip_header=1)
	# data[:,1:] *= 0.001 #convert pose data to meters

	# print(data)
	return data

# return the corner points of tags relative to absolute zero
def parse_tag_points(pointsFile):
	data =  np.genfromtxt(pointsFile, delimiter=',',skip_header=1)
	# data[:,1:] *= 0.001 #convert pose data to meters

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
	tag_poses = parse_tag_centroids(args.mocaptagsfile)
	aruco_data = parse_aruco_data(args.detectmarkersfile, args.timestampsfile)

	errors = compute_error(aruco_data, robot_pose, tag_poses)

	plt.figure(figsize=(15,15), facecolor='black')

	colors = cm.rainbow(np.linspace(0, 1, tag_poses.shape[0]))

	c_data = []
	for i in range(errors.shape[0]):
		idx =  np.where(tag_poses[:,0] == int(errors[i,1]) )
		c_data.append( colors[idx[0][0]] )
		# c_data.append( hex2rgb( colors[idx[0][0]] ))

    # remove bad estimates
	i = 0
	temp = []
	while(i < aruco_data.shape[0]):
		if( aruco_data[i,2] < -999998.):
			pass
		else:
			temp.append(aruco_data[i,:])
		i += 1

    # plot pose data only for tag 4
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

	robot_pose[:,1] -= tag_poses[2,1]
	robot_pose[:,2] -= tag_poses[2,2]
	robot_pose[:,3] -= tag_poses[2,3]

	robot_pose[:,1] *= 0.001
	robot_pose[:,2] *= 0.001
	robot_pose[:,3] *= 0.001

	colors = cm.rainbow(np.linspace(0, 1, aruco_data.shape[0]))

	plt.scatter(aruco_data[:,0], aruco_data[:,4], s=10, color='g')
	plt.scatter(robot_pose[:,0], robot_pose[:,2], s=10, color='r')

	# plt.scatter(aruco_data[:,0], aruco_data[:,3], s=10, color='g')
	# plt.scatter(robot_pose[:,0], robot_pose[:,3], s=10, color='r')

	# plt.scatter(aruco_data[:,0], aruco_data[:,2], s=10, color='g')
	# plt.scatter(robot_pose[:,0], robot_pose[:,1], s=10, color='r')
	# plt.scatter(aruco_data[:,4], aruco_data[:,2], s=10, color=colors)


	plt.title("Y distance from Tag 4 (aruco vs mocap) over time", color='white')
	plt.ylabel("Robot Y (meters)", color='white')
	plt.xlabel("time (seconds)", color='white')
	plt.tick_params(colors='white')

	plt.show()

def computeNormalVec(p1, p2, p3):
	v1 = np.array([p2[i] - p1[i] for i in range(3) ])
	v2 = np.array([p3[i] - p1[i] for i in range(3) ])
	v1_x_v2 = np.cross(v1,v2)

	# radius of vector
	r = np.sqrt( v1_x_v2[0]**2 + v1_x_v2[1]**2 + v1_x_v2[2]**2 )
	# angle in x-y plane
	t = np.arctan2(v1_x_v2[1], v1_x_v2[0])
	# angle away from z axis
	p = np.arccos(v1_x_v2[2]/r)

	return v1_x_v2/r

# return x y z numpy array
def computeRotation(p1, p2, p3):
    # print(p1)
    # print(p2)
    # print(p3)
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

    # print("Cross v1_x_v2, v2",v3)
    # print([r, t,  p])
    return [r, t, p]


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

	# print(temp)
	return temp

def plotMarkerMapMocap():
	robot_pose = parse_bot_pose(args.mocaprobotfile)
	tag_poses = parse_tag_centroids(args.mocaptagsfile)
	aruco_data = parse_aruco_data(args.detectmarkersfile, args.timestampsfile)
	errors = compute_error(aruco_data, robot_pose, tag_poses)
	tag_points = parse_tag_points(args.tagpointsfile)

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

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	points = getPointPoses(tag_poses, tag_points)

	points *= .001

	print(points[0,0:3].shape)
	print(points[0,3:6].shape)
	print(points[0,5:8].shape)
	print(points[0,:])
	print(points[-1,:])

	print("Rot 0 ", computeRotation(points[0,0:3], points[0,3:6], points[0,5:8]))

	print(points.shape)

	set_axes_equal(ax)

	points *= 1000

	# top left
	ax.scatter(points[:,0], points[:,1], points[:,2], s=10, color='r')
	# right
	ax.scatter(points[:,3], points[:,4], points[:,5], s=10, color='g')
	# bottom left
	ax.scatter(points[:,6], points[:,7], points[:,8], s=10, color='b')

	plt.show()

# stolen from Peter
def get_center_tag_poses_from_dots(top_left_dot_poses, dot_offsets):
    true_map = np.zeros((top_left_dot_poses.shape[0], 13))
    dots = np.zeros((top_left_dot_poses.shape[0] * 3, 3))
    ids = top_left_dot_poses[:, 0]

    j = 0
    for i, (top_left_dot_pose, dot_offset, tag_id) in enumerate(zip(top_left_dot_poses, dot_offsets, ids)):
        top_left_dot = top_left_dot_pose[1:4]
        centroid = top_left_dot - dot_offset[1:4]
        right_dot_pose = centroid + dot_offset[4:7]
        bottom_left_dot_pose = centroid + dot_offset[7:]

        dots[j] = top_left_dot / 1000
        dots[j + 1] = right_dot_pose / 1000
        dots[j + 2] = bottom_left_dot_pose / 1000

        # create the y axis in the tag frame as defined by aruco
        tag_y = top_left_dot - bottom_left_dot_pose
        tag_y = tag_y / np.linalg.norm(tag_y)

        # compute the normal vector of the plane those three points lie in
        top_left_to_right = right_dot_pose - top_left_dot
        tag_z = np.cross(top_left_to_right, tag_y)
        tag_z = tag_z / np.linalg.norm(tag_z)

        # now find the axis orthogonal to both the one going from top-left to bottom-left and the normal
        tag_x = np.cross(tag_y, tag_z)
        tag_x = tag_x / np.linalg.norm(tag_x)

        tag_size_mm = 89.2
        tag_center = top_left_dot + (26.87 + tag_size_mm / 2.0) * tag_x + (-63.17 - tag_size_mm / 2.0) * tag_y
        true_map[i, 0:3] = tag_center / 1000  # convert to meters
        true_map[i, 3:6] = tag_x
        true_map[i, 6:9] = tag_y
        true_map[i, 9:12] = tag_z
        true_map[i, 12] = tag_id

        j += 3

    return true_map, dots, ids

def transform_map(aruco_pose, true_map):
	origin = true_map[aruco_pose[0],:] #pose of tag in MoCap Space
	aruco_pose = aruco_pose[1:]
	new_pose = np.copy(aruco_pose)


# https://stackoverflow.com/questions/6802577/rotation-of-3d-vector
def rotation_matrix(axis, theta):
	axis = np.asarray(axis)
	axis = axis/math.sqrt(np.dot(axis, axis))
	a = math.cos(theta/2.0)
	b, c, d = -axis*math.sin(theta/2.0)
	aa, bb, cc, dd = a*a, b*b, c*c, d*d
	bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
	return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
	                 [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
	                 [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

def angleError():

	aruco_transforms = parse_aruco_data(args.detectmarkersfile, args.timestampsfile)
	robot_poses = parse_bot_pose(args.mocaprobotfile)
	tag_poses = parse_tag_centroids(args.mocaptagsfile)
	tag_offsets = parse_tag_points(args.tagpointsfile)
	tag_centers = get_center_tag_poses_from_dots(tag_poses, tag_offsets)

	error = []
	# for each tag_detected/timestamp pair, find the closest (in terms of time)
	# mocap frame and compute the error along each axis, store in list with timestamp

	# for i in range(aruco_transforms.shape[0]):
	# 	# get index of closest timestamp
	# 	tsIdxT = (np.abs(robot_poses[:,0]-aruco_transforms[i,0])).argmin()

	# 	robot_pose = robot_poses[tsIdxT,:]

	# 	tag_id = int(aruco_transforms[i,1])
	# 	tag_pose = tag_centers[0][np.where(tag_centers[0][:,12] == tag_id)[0][0],:-1]

	# 	theta = robot_pose[-1]
	# 	xRotOffset = -3.14/2.

	# 	A = tag_pose[3:13].reshape((3,3)).T
	# 	B = np.array( [[np.cos(theta), -np.sin(theta), 0 ], [np.sin(theta), np.cos(theta), 0 ], [0, 0, 1]] )
	# 	C = np.array( [ [1, 0, 0], [0, np.cos(xRotOffset), np.sin(xRotOffset) ], [0, -np.sin(xRotOffset), np.cos(xRotOffset)] ] )

	# 	robot_rot = C@B@np.linalg.inv(A)

	# 	aruco_rot = np.linalg.norm(aruco_transforms[i,5:8])

def calc_trans():
	aruco_data, timestamps = parse_aruco_data(args.detectmarkersfile, args.timestampsfile)
	robot_pose = parse_bot_pose(args.mocaprobotfile)
	tag_poses = parse_tag_centroids(args.mocaptagsfile)
	tag_offsets = parse_tag_points(args.tagpointsfile)
	tag_centers = get_center_tag_poses_from_dots(tag_poses, tag_offsets)

	start_ts = timestamps[0]
	end_ts = timestamps[-1]

	# get first tag detected (Tag 4), find corresponding mocap data
	aruco_first_sample = aruco_data[0,:]
	tag_id = aruco_first_sample[1]
	first_detection_ts = aruco_first_sample[0]

	corresponding_mocap_idx = (np.abs(robot_pose[:,0]-first_detection_ts)).argmin()

	robot_pose_first_ts = robot_pose[corresponding_mocap_idx,:]

	tag_pose = tag_centers[0][np.where(tag_centers[0][:,12] == tag_id)[0][0],:-1]
	tag_id_ = tag_centers[0][np.where(tag_centers[0][:,12] == tag_id)[0][0],-1]

	# print(tag_id)
	print(robot_pose_first_ts)
	# print(aruco_first_sample)
	print(tag_pose, tag_id_)

	A = tag_pose[3:13].reshape((3,3)).T

	robot_z = robot_pose_first_ts[-1]

	xOffset = (-3.14/2) 

	B = np.array( [	[np.cos(robot_z), -np.sin(robot_z), 0 ], 
									[np.sin(robot_z), np.cos(robot_z), 0 ], 
									[0, 0, 1]] )

	C = np.array( [ [1, 0, 0], 
									[0, np.cos(xOffset), np.sin(xOffset) ], 
									[0, -np.sin(xOffset), np.cos(xOffset)] ] )

	mocap_rot = C@B@np.linalg.inv(A)

	aruco_rod_theta = np.linalg.norm(aruco_first_sample[5:8])
	unit_vec = aruco_first_sample[5:8]/aruco_rod_theta
	aruco_rot = rotation_matrix(unit_vec, aruco_rod_theta)

	print(aruco_rot)
	print(mocap_rot)

	# get translation in mocap frame
	# account for offset between camera and vicon trackers
	t = (robot_pose_first_ts[1:4] - np.array([0, 0, .170]) - tag_pose[0:3]) 

	print("mocap frame transalation \n",t)
	print("aruco frame transalation \n",aruco_first_sample[2:5])

	# translation mocap in aruco frame
	trans = np.eye(3)@np.linalg.inv(A)@t.T
	
	print("mocap translation aruco frame\n",trans)
	print(np.linalg.norm(trans) - np.linalg.norm(aruco_first_sample[2:5]))
	print(np.linalg.norm(t) - np.linalg.norm(aruco_first_sample[2:5]))

	mocap_norm = np.linalg.norm(t)
	aruco_norm = np.linalg.norm(aruco_first_sample[2:5])
	
	print(mocap_norm - aruco_norm)
	error = trans - aruco_first_sample[2:5]
	print(error)


def plotArucoPoseEstimate():
	aruco_data = parse_aruco_data(args.detectmarkersfile, args.timestampsfile)
	robot_pose = parse_bot_pose(args.mocaprobotfile)
	tag_poses = parse_tag_centroids(args.mocaptagsfile)

	end_diff = robot_pose[-1,0] - aruco_data[-1,0]

	tag_id = 1

	tag_poses[:,1] +=  0.06317
	tag_poses[:,2] +=  0.02687

	disp_tag_4 = robot_pose
	disp_tag_4[:,1] -= tag_poses[tag_id, 1]
	disp_tag_4[:,2] -= tag_poses[tag_id, 2]
	disp_tag_4[:,3] -= tag_poses[tag_id, 3]

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

	# get only data for tag ID 4
	temp = []
	for i in range(aruco_data.shape[0]):
		idx =  np.where(tag_poses[tag_id,0] == int(aruco_data[i,1]) )
		if(idx[0].shape[0] == 0):
			pass
		else:
			temp.append( aruco_data[i,:] )

	aruco_data = np.array(temp)

	aruco_data[:,0] += end_diff

	# aruco_data = aruco_data[1::10]
	fig = plt.figure()
	# ax = fig.add_subplot(111, projection='3d')
	ax = fig.add_subplot(111)
	LIM = 4.0
	# ax.set_xlim3d(0, LIM)
	# ax.set_ylim3d(0,LIM)
	# ax.set_zlim3d(0,LIM)
	# set_axes_equal(ax)

	# print(aruco_data[-1,0] - robot_pose[-1,0])

	reds = cm.Reds(np.linspace(1, .3, aruco_data.shape[0]))
	blues = cm.Blues(np.linspace(1, .3, aruco_data.shape[0]))
	greens = cm.Greens(np.linspace(1, .3, aruco_data.shape[0]))

	# ax.scatter(aruco_data[:,2], aruco_data[:,3], aruco_data[:,4], color=colors)
	ax.scatter(aruco_data[:,0], aruco_data[:,2], color=reds, label='Aruco X')
	ax.scatter(aruco_data[:,0], aruco_data[:,3], color=blues, label='Aruco Y')
	ax.scatter(aruco_data[:,0], aruco_data[:,4], color=greens, label='Aruco Z')

	autumn = cm.autumn(np.linspace(0, .8, disp_tag_4.shape[0]))
	summer = cm.summer(np.linspace(0, .8, disp_tag_4.shape[0]))
	winter = cm.winter(np.linspace(0, .8, disp_tag_4.shape[0]))

	# ax.scatter(disp_tag_4[:,1], disp_tag_4[:,2], disp_tag_4[:,3], color=colors)
	ax.scatter(disp_tag_4[:,0], disp_tag_4[:,1], color=autumn, label='Vicon X')
	ax.scatter(disp_tag_4[:,0], disp_tag_4[:,2], color=summer, label='Vicon Y')
	ax.scatter(disp_tag_4[:,0], disp_tag_4[:,3], color=winter, label='Vicon Z')

	ax.legend()
	ax.set_title("Distance to Tag " + str(int(tag_poses[tag_id,0])) + " over Time")
	ax.set_xlabel("Time (seconds)")
	ax.set_ylabel("Distance (meters)")
	ax.legend(scatterpoints=1)
	# ax.quiver(aruco_data[:, 4], aruco_data[:, 2], aruco_data[:, 3], aruco_data[:, 7], aruco_data[:, 5], aruco_data[:, 6], length=0.3, arrow_length_ratio=0.5, color='red')
	plt.show()

def main():
	calc_trans()

if __name__ == '__main__':
	sys.exit(main())
