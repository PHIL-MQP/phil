#!/usr/bin/env python

import argparse
import csv
import os

import matplotlib
import matplotlib.pyplot as plt
import numpy as np


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
        robot_pose = np.zeros(len(mocap_data))
        for i, d in enumerate(mocap_data):
            try:
                robot_pose[i] = float(d)
            except ValueError:
                pass  # default to 0
        robot_poses.append(robot_pose[robot_columns])
    robot_poses = np.array(robot_poses)
    return robot_poses


def load_rio_data(rio_reader):
    data = []
    next(rio_reader)
    for row in rio_reader:
        data.append([float(d) for d in row])
    return np.array(data)


def integrate_gyro_z(gyro_z):
    integrated_yaws = np.zeros(len(gyro_z))
    yaw = 0
    for i, g in enumerate(gyro_z):
        yaw += g * 0.02
        if yaw > 180:
            yaw = -180
        elif yaw < -180:
            yaw = 180
        integrated_yaws[i] = yaw
    return integrated_yaws


def wrap_mocap(mocap_yaws):
    yaws = np.zeros(mocap_yaws.shape)
    for i, y in enumerate(mocap_yaws):
        if y < -180:
            yaws[i] = y + 360
        elif y > 180:
            yaws[i] = y - 360
        else:
            yaws[i] = y
    return yaws


def main():
    default_robot_name = 'Global Angle phil_turtlebot:phil_turtlebot'

    parser = argparse.ArgumentParser("This script compares the yaw measurements of the fused navx reading, "
                                     "naively integrated gyroscope, and motion capture.")
    parser.add_argument('mocap_csv', help='csv motion capture file')
    parser.add_argument('rio_csv', help='csv of navx data from the roborio')
    parser.add_argument('--robot-name',
                        help='title of the column with the rio data. default is [%s]'.format(default_robot_name),
                        default=default_robot_name)
    parser.add_argument("--no-plot", '-p', action="store_true", help="skip plotting")
    parser.add_argument("--save", '-s', action="store_true", help="save to yaw_comparison.png")

    args = parser.parse_args()

    mocap_reader = csv.reader(open(args.mocap_csv, 'r'))
    rio_reader = csv.reader(open(args.rio_csv, 'r'))

    # MOCAP data
    robot_poses = load_motion_capture_csv(args.robot_name, mocap_reader)
    if robot_poses is None:
        print("there's no column [{:s}] in [{:s}]".format(args.robot_name, args.mocap_csv))
        return
    mocap_yaws = robot_poses[:, 2] - robot_poses[0, 2]
    mocap_yaws = robot_poses[:, 2] - robot_poses[1, 2]
    mocap_yaws = np.rad2deg(mocap_yaws)
    mocap_yaws = wrap_mocap(mocap_yaws)
    mocap_times = np.arange(0, mocap_yaws.shape[0]) * 0.01

    # RIO data
    rio_data = load_rio_data(rio_reader)
    navx_yaws = -rio_data[:, 8]
    navx_yaws = navx_yaws
    navx_times = (rio_data[:, -1].astype(np.int32) - rio_data[0, -1]) / 1000
    gyro_z = rio_data[:, 5]

    integrated_yaws = integrate_gyro_z(gyro_z)

    # compute the average error of each data source
    time_conversion = mocap_times.shape[0] / navx_times.shape[0]
    nearest_mocap_times = np.arange(navx_times.shape[0]) * time_conversion

    navx_errors = []
    integrated_errors = []
    outliers = 0
    nearest_mocap_yaws = []
    for mocap_time, navx_yaw, integrated_yaw in zip(nearest_mocap_times, navx_yaws, integrated_yaws):
        lower_idx = int(np.floor(mocap_time))
        upper_idx = int(np.ceil(mocap_time))
        mocap_yaw = (mocap_yaws[lower_idx] + mocap_yaws[upper_idx])/2
        nearest_mocap_yaws.append(mocap_yaw)
        navx_error = abs(navx_yaw - mocap_yaw)
        integrated_error = abs(integrated_yaw - mocap_yaw)
        if navx_error < 40: # ignore outliers
            navx_errors.append(navx_error)
        else:
            outliers += 1
        if integrated_error < 40: # ignore outliers
            integrated_errors.append(integrated_error)
        else:
            outliers += 1

    print("Found {:d} outliers".format(outliers))
    print("Data Source & Average Error (deg) & 90th Percentile Error (deg) \\\\ \hline")
    print("Navx \\texttt{{GetYaw()}} & {:0.3f} & {:0.3f} \\\\ \hline".format(np.mean(navx_errors), np.percentile(navx_errors, 90)))
    print("Integrated & {:0.3f} & {:0.3f} \\\\ \hline".format(np.mean(integrated_errors), np.percentile(integrated_errors, 90)))


    if not args.no_plot:
        if 'DISPLAY' not in os.environ:
            matplotlib.use('Agg')

        recorded_data_processing_dir = os.path.dirname(os.path.realpath(__file__))
        style = recorded_data_processing_dir + "/../phil.mplstyle"
        plt.style.use(style)
        plt.figure()
        plt.scatter(mocap_times, mocap_yaws, s=1, label='motion capture', lw=0)
        plt.scatter(navx_times, navx_yaws, s=1, label='NavX getYaw()', lw=0)
        plt.scatter(navx_times, integrated_yaws, s=1, label='Integrated Gyro Z', lw=0)
        plt.ylabel("Yaw (degrees)")
        plt.xlabel("Time (seconds)")
        plt.title("Comparison of Yaw Measurement")
        legend = plt.legend()
        legend.legendHandles[0]._sizes = [30]
        legend.legendHandles[1]._sizes = [30]
        legend.legendHandles[2]._sizes = [30]
        plt.show()


if __name__ == '__main__':
    main()
