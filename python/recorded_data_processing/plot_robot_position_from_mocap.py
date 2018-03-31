#!/usr/bin/env python

import argparse
import csv
import os

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm


def load_motion_capture_csv(robot_name, mocap_reader):
    """ each row is RX,RY,RZ,TX,TY,TZ """
    next(mocap_reader)
    next(mocap_reader)
    object_names = next(mocap_reader)
    next(mocap_reader)
    next(mocap_reader)

    if robot_name not in object_names:
        return None

    robot_columns = np.arange(6, dtype=np.int32) + int(object_names.index(robot_name))
    robot_poses = []
    for mocap_data in mocap_reader:
        if not mocap_data:
            break
        robot_pose = np.zeros(len(mocap_data))
        for i, d in enumerate(mocap_data):
            try:
                robot_pose[i] = float(d)
            except ValueError:
                pass  # default to 0
        robot_poses.append(robot_pose[robot_columns])
    robot_poses = np.array(robot_poses)
    return robot_poses


def main():
    default_robot_name = 'Global Angle phil_turtlebot:phil_turtlebot'

    parser = argparse.ArgumentParser("plot the X/Y/Yaw of the robot from a mocap file")
    parser.add_argument('mocap_csv', help='csv motion capture file')
    parser.add_argument('--robot-name',
                        help='title of the column with the rio data. default is [%s]'.format(default_robot_name),
                        default=default_robot_name)
    parser.add_argument("--tracker-to-robot-yaw", help="offset in radians between the tracker and the robot",
                        type=float, default=0)

    args = parser.parse_args()

    mocap_reader = csv.reader(open(args.mocap_csv, 'r'))

    # MOCAP data
    robot_poses = load_motion_capture_csv(args.robot_name, mocap_reader)
    if robot_poses is None:
        print("there's no column [{:s}] in [{:s}]".format(args.robot_name, args.mocap_csv))
        return

    recorded_data_processing_dir = os.path.dirname(os.path.realpath(__file__))
    style = recorded_data_processing_dir + "/../phil.mplstyle"
    plt.style.use(style)
    skip = 10
    # this is just a hack because we don't know the offset between robot "forward" and the tracking markers on it
    yaws = robot_poses[::skip, 2] + args.tracker_to_robot_yaw

    plt.figure()
    colors = cm.rainbow(np.linspace(0, 1, robot_poses.shape[0]))
    plt.scatter(robot_poses[:, 3]/1000, robot_poses[:, 4]/1000, s=1, c=colors)
    plt.quiver(robot_poses[::skip, 3]/1000, robot_poses[::skip, 4]/1000, np.cos(yaws), np.sin(yaws))
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis("square")
    plt.title("Position of Robot form MoCap")
    plt.show()


if __name__ == '__main__':
    main()
