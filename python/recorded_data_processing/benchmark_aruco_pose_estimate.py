#!/usr/bin/python3

# x -> 0.06317
# y -> 0.02687
# y displcement between tags --> 121 mm 

# turtlebot measures about 45-46 cm tall
# measured offset between camera and turtlebot = 17 cm

'''
python3 recorded_data_processing/benchmark_aruco_pose_estimate.py ../cpp/build/4_16_trial1_3_17/detect_markers_out.csv ../cpp/build/video1_03_17_11-16-14/timestamps.csv ../cpp/build/4_16_trial1_3_17/robot_poses.csv ../cpp/build/4_8_aruco/tag_centroid_poses.csv ../cpp/build/4_8_aruco/tag_corner_poses.csv

Trial 0:
Mean error distance 0.111263917549
St dev distance 0.144300438127
pct95 error distance 0.523438322996
pct5 dev distance 0.00671486483371

Trial 1:
Mean error distance 0.112630790696
St dev distance 0.128490115455
pct95 error distance 0.376928272793
pct5 dev distance 0.00817272854998


'''

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
parser.add_argument("timestampsfile", help="The timestamps file for video stream")
parser.add_argument("mocaprobotfile", help="The robot poses from vicon")
parser.add_argument("mocaptagstopleftfile", help="The hand-written tag top left position of tags from vicon")
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

# return the top left corners of tags as coordinates from abs zero of the mocap system
def parse_tag_top_left(tagfile):
  data = np.genfromtxt(tagfile, delimiter=',',skip_header=1)
  # data[:,1:] *= 0.001 #convert pose data to meters

  # print(data)
  return data

# return the corner points of tags relative to absolute zero
def parse_tag_points(pointsFile):
  data =  np.genfromtxt(pointsFile, delimiter=',',skip_header=1)
  # data[:,1:] *= 0.001 #convert pose data to meters

  return data

def compute_dropped_poses(aruco):
  errors = []
  bad_pose = 0

  for i in range(aruco.shape[0]):
    if(aruco[i,2] == -999999.):
      bad_pose += 1

  print("Bad Poses", str(bad_pose))
  print("Total Detected", str(aruco.shape[0]))
  print("Dropped Pct", str(bad_pose*100./float(aruco.shape[0])))
  return bad_pose/aruco.shape[0]

# stolen from Peter
def get_center_tag_poses_from_dots(top_left_dot_poses, dot_offsets):
    if(top_left_dot_poses.shape[0] != dot_offsets.shape[0]):
      print("Suspicious! the files should have the same number of rows. exiting")
      return
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

def dropped_poses():
    aruco_data, ts = parse_aruco_data(args.detectmarkersfile, args.timestampsfile)
    compute_dropped_poses(aruco_data)

# finds the transform beetween the mocap frame and frame of the tag,
# compute the rotational matrix of the ArUco pose estimate, and compare it to the 
# mocap data
def compute_angle_error():
  # parse data from files
  aruco_data, timestamps = parse_aruco_data(args.detectmarkersfile, args.timestampsfile)
  robot_pose = parse_bot_pose(args.mocaprobotfile)
  tag_poses = parse_tag_top_left(args.mocaptagstopleftfile)
  tag_offsets = parse_tag_points(args.tagpointsfile)
  tag_centers = get_center_tag_poses_from_dots(tag_poses, tag_offsets)

  start_ts = timestamps[0]
  end_ts = timestamps[-1]

  # get first tag detected, find corresponding mocap data
  aruco_first_sample = aruco_data[0,:]
  tag_id = aruco_first_sample[1]
  first_detection_ts = aruco_first_sample[0]

  corresponding_mocap_idx = (np.abs(robot_pose[:,0]-first_detection_ts)).argmin()

  robot_pose_first_ts = robot_pose[corresponding_mocap_idx,:]

  tag_pose = tag_centers[0][np.where(tag_centers[0][:,12] == tag_id)[0][0],:-1]

  A = tag_pose[3:13].reshape((3,3)).T

  robot_z = robot_pose_first_ts[-1]

  xOffset = (-3.14/2)

  B = np.array( [  [np.cos(robot_z), -np.sin(robot_z), 0 ], 
                  [np.sin(robot_z), np.cos(robot_z), 0 ], 
                  [0, 0, 1]] )

  C = np.array( [ [1, 0, 0], 
                  [0, np.cos(xOffset), np.sin(xOffset) ], 
                  [0, -np.sin(xOffset), np.cos(xOffset)] ] )

  mocap_rot = A@np.linalg.inv(B)@np.linalg.inv(C)
  aruco_rod_theta = np.linalg.norm(aruco_first_sample[5:8])
  unit_vec = aruco_first_sample[5:8]/aruco_rod_theta
  aruco_rot = rotation_matrix(unit_vec, aruco_rod_theta)

  print("Mocap", mocap_rot)
  print()
  print("ArUco", aruco_rot)


def compute_distance_error(plot=False):
  # parse data from files
  aruco_data, timestamps = parse_aruco_data(args.detectmarkersfile, args.timestampsfile)
  robot_pose = parse_bot_pose(args.mocaprobotfile)
  tag_poses = parse_tag_top_left(args.mocaptagstopleftfile)
  tag_offsets = parse_tag_points(args.tagpointsfile)
  tag_centers = get_center_tag_poses_from_dots(tag_poses, tag_offsets)

  # apply time shift to account for delay in camera start-up
  aruco_data[:,0] += 0.55

  output = []

  for i in range(aruco_data.shape[0]):
    temp = []
    aruco_sample = aruco_data[i,:]
    tag_id = aruco_sample[1]
    timestamp = aruco_sample[0]

    corresponding_mocap_idx = (np.abs(robot_pose[:,0]-timestamp)).argmin()

    robot_pose_temp = robot_pose[corresponding_mocap_idx,:]

    try:
      tag_pose = tag_centers[0][np.where(tag_centers[0][:,12] == tag_id)[0][0],:-1]
    except Exception as e:
      continue
    else:
      pass
    finally:
      pass

    mocap_robot_to_tag = robot_pose_temp[1:4] - np.array([0, 0, .17]) - tag_pose[0:3]

    # timestamp (aruco), tag id, truth (mocap), measured (aruco)
    temp.append(timestamp)
    temp.append(tag_id)
    temp.append(np.linalg.norm(mocap_robot_to_tag))
    temp.append(np.linalg.norm(aruco_sample[2:5]))
    temp.append(np.abs(np.linalg.norm(mocap_robot_to_tag) - np.linalg.norm(aruco_sample[2:5]))) # error
    temp.append(corresponding_mocap_idx)
    output.append(temp)

  output = np.array(output)

  mean_dist = np.mean(output[:,4])
  st_dev_dist = np.std(output[:,4])
  pct95 = np.percentile(output[:,4], 95)
  pct5 = np.percentile(output[:,4], 5)

  print("Mean error distance",mean_dist)
  print("St dev distance",st_dev_dist)
  print("pct95 error distance",pct95)
  print("pct5 dev distance",pct5)

  if(plot == True):
    plt.figure(figsize=(15,15), facecolor='black')

    colors_vicon = cm.Accent(np.linspace(0, 1, tag_poses.shape[0]))
    colors_aruco = cm.Set1(np.linspace(0, 1, tag_poses.shape[0]))

    for i in range(tag_centers[0].shape[0]):
      try:
        mean_errors_idxs = np.where(output[:,1] == tag_centers[0][i,12])[0]
      except Exception as e:
        continue
      if(mean_errors_idxs.shape[0] > 0):
        errors_by_tag = []
        temp = []
        for err in mean_errors_idxs:
          errors_by_tag.append(output[err,4])
          temp.append(output[err,:])
        temp = np.array(temp)
        labelV =  "Vicon " + str(int(tag_centers[0][i,12]))
        labelA = "ArUco " + str(int(tag_centers[0][i,12]))
        idx =  np.where(tag_poses[:,0] == int(tag_centers[0][i,12]) )
        plt.scatter(temp[:,0], temp[:,2], s=10, color=colors_vicon[idx[0][0]], label=labelV)
        plt.scatter(temp[:,0], temp[:,3], s=10, color=colors_aruco[idx[0][0]], label=labelA)

    # print(np.mean(errors_by_tag))
    # print(tag_centers[0][i,12] )

  # plot corresponding_mocap_idx over time
  # plt.scatter(output[:,0], output[:,5], s=10, color="g") 

    lgnd = plt.legend(scatterpoints=1, fontsize=16)
    for i in range(len(lgnd.legendHandles)):
      lgnd.legendHandles[i]._sizes = [40]
    plt.title("Distance from Camera to Tag (ArUco vs Vicon) Trial 1", color='white', fontsize=20)
    plt.ylabel("Distance (meters)", color='white', fontsize=20)
    plt.xlabel("time (seconds)", color='white',fontsize=20)
    plt.tick_params(colors='white')
    plt.show()

def main():
  compute_distance_error(plot=True)

if __name__ == '__main__':
  sys.exit(main())
