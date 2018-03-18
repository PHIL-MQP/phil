#!/usr/bin/env python

import argparse
import csv
import os

import matplotlib
import matplotlib.pyplot as plt
import numpy as np


def main():
    default_robot_name = 'Global Angle phil_turtlebot:phil_turtlebot'

    parser = argparse.ArgumentParser("This script compares the yaw measurements of the fused navx reading, "
                                     "naively integrated gyroscope, and motion capture.")
    parser.add_argument('mocap_csv', help='csv motion capture file')
    parser.add_argument('rio_csv', help='csv of navx data from the roborio')
    parser.add_argument('--robot_name',
                        help='title of the column with the rio data. default is [%s]'.format(default_robot_name),
                        default=default_robot_name)
    parser.add_argument("--no-plot", '-p', action="store_true", help="plot FPS over time")

    args = parser.parse_args()

    mocap_reader = csv.reader(open(args.mocap_csv, 'r'))
    rio_reader = csv.reader(open(args.rio_csv, 'r'))

    robot_poses = load_motion_capture_csv(args.robot_name, mocap_reader)
    rio_data = load_rio_data(rio_reader)
    navx_yaws = rio_data[:,8]

    if robot_poses is None:
        print("there's no column [{:s}] in [{:s}]".format(args.robot_name, args.mocap_csv))

    mocap_yaws = robot_poses[:, 2]
    mocap_times = np.arange(0, mocap_yaws.shape[0]) * 0.01

    if not args.no_plot:
        if 'DISPLAY' not in os.environ:
            matplotlib.use('Agg')

        recorded_data_processing_dir = os.path.dirname(os.path.realpath(__file__))
        style = recorded_data_processing_dir + "/../phil.mplstyle"
        plt.style.use(style)
        plt.figure()
        plt.plot(mocap_times, mocap_yaws, linewidth=1, label='motion capture')
        plt.plot(mocap_times, navx_yaws, linewidth=1, label='NavX getYaw()')
        plt.ylabel("Yaw (degrees)")
        plt.xlabel("Time (seconds)")
        plt.title("Comparison of Yaw Measurement")
        plt.show()


def load_motion_capture_csv(robot_name, mocap_reader):
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
        robot_pose = np.array([float(d) for d in mocap_data])
        robot_poses.append(robot_pose[robot_columns])
    robot_poses = np.array(robot_poses)
    return robot_poses


def load_rio_data(rio_reader):
    data = []
    next(rio_reader)
    for row in rio_reader:
        print(row)
        data.append([float(d) for d in row])
    return np.array(data)


if __name__ == '__main__':
    main()
