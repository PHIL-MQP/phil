#!/usr/bin/python3

import numpy as np
import sys
import matplotlib.pyplot as plt
import csv
import matplotlib.cm as cm
from matplotlib.patches import Ellipse, Circle

import argparse

np.set_printoptions(suppress=True)  # no scientific notation


parser = argparse.ArgumentParser("Compute error of aruco estimate pose")
parser.add_argument("detectmarkersfile", help="The detect markers output")
parser.add_argument("mocaptagsfile", help="The hand-written tag poses from vicon")
parser.add_argument("mocaprobotfile", help="The robot poses from vicon")
parser.add_argument("timestampsfile", help="The timestamps file for video stream")

args = parser.parse_args()

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
		for j in range(tag.shape[1]):
			if( tag[j,0] == aruco[i,1] ):
				if(aruco[i,2] == -999999.):
					bad_pose += 1
					continue
				# find the robot pose that matches this timestamp
				tsIdxT = (np.abs(robot[:,0]-aruco[i,0])).argmin()

				mocap_tf = np.array([robot[tsIdxT,1] - tag[j,1], # x
										robot[tsIdxT,3] - tag[j,3], # y and z are swapped
										robot[tsIdxT,2] - tag[j,2] ]) # z and y are swapped

				mocap_tf *= 0.001
				# print(mocap_tf)
				# print(aruco[i,2:5])
				error = np.abs(mocap_tf - aruco[i,2:5])
				error = np.array([aruco[i,0], aruco[i,1], error[0], error[1], error[2]])
				errors.append(error)
				# print(error)
				# print(bad_pose)
	errors = np.array(errors)
	for e in errors:
		print(e)
	return errors

def main():
	robot_pose = parse_bot_pose(args.mocaprobotfile)
	tag_poses = parse_tag_poses(args.mocaptagsfile)
	aruco_data = parse_aruco_data(args.detectmarkersfile, args.timestampsfile)

	errors = compute_error(aruco_data, robot_pose, tag_poses)

	mean_x = np.mean(errors[2,:])
	med_x = np.median(errors[2,:])
	pct95_x = np.percentile(errors[2:], 95)
	pct5_x = np.percentile(errors[2:], 5)

	print(mean_x, med_x, pct95_x, pct5_x)
	plt.figure(figsize=(15,15), facecolor='black')
	colors = cm.ocean(np.linspace(0, 1, len(errors)))
	plt.scatter(errors[:,2], errors[:,3], s=4, color=colors)
	plt.ylabel("Error Y (meters)", color='white')
	plt.xlabel("Error X (seconds)", color='white')
	plt.title("X Y Error from all Tags (position)", color='white')
	plt.tick_params(colors='white')
	# plt.axis("equal")
	plt.show()
	# plt.show()


if __name__ == '__main__':
    sys.exit(main())
