#!/usr/bin/env python3

import argparse
import csv
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from recorded_data_processing.plot_robot_position_from_mocap import load_motion_capture_csv


def main():
    default_robot_name = 'Global Angle PHIL_ROBOT_01:PHIL_ROBOT_01'

    parser = argparse.ArgumentParser("run forward kinematics on encoder data from CSV")
    parser.add_argument('csv_file', help='csv file with some columns of encoder data')
    parser.add_argument('mocap_csv', help='csv motion capture file')
    parser.add_argument('--robot-name',
                        help='title of the column with the rio data. default is [%s]'.format(default_robot_name),
                        default=default_robot_name)
    robots = parser.add_mutually_exclusive_group(required=True)
    robots.add_argument('--turtlebot', action='store_true')
    robots.add_argument('--mocapbot', action='store_true')
    parser.add_argument('--alpha', help='slip value, must be >=1', default=1, type=float)
    parser.add_argument('--initial-yaw', help='initial yaw in degrees', default=0, type=float)

    args = parser.parse_args()

    mocap_reader = csv.reader(open(args.mocap_csv, 'r'))

    # MOCAP data
    robot_poses = load_motion_capture_csv(args.robot_name, mocap_reader)
    if robot_poses is None:
        print("there's no column [{:s}] in [{:s}]".format(args.robot_name, args.mocap_csv))
        return
    mocap_xs = (robot_poses[:, 3] - robot_poses[0, 3])/1000
    mocap_ys = (robot_poses[:, 4] - robot_poses[0, 4])/1000

    if args.turtlebot:
        track_width_m = 0.23
        wheel_radius_m = 0.035
        step = 6
        dt_s = 0.023 * step
        ticks_per_rev = 2578.33
    else:
        track_width_m = 0.9
        wheel_radius_m = 0.074
        dt_s = 0.02
        distance_per_tick = 0.01375

    data = np.genfromtxt(args.csv_file, delimiter=',', skip_header=True)

    wheel_speeds_rps = []
    if args.turtlebot:  # turtlebot gets ticks.
        left_ticks = data[:, 0]
        right_ticks = data[:, 1]
        left_ticks = np.unwrap(left_ticks, 2 ** 8)
        right_ticks = np.unwrap(right_ticks, 2 ** 8)

        for i in range(0, data.shape[0] - step, step):
            _dt_s = (data[i + step, 2] + data[i + step, 3] / 1e9) - (data[i, 2] + data[i, 3] / 1e9)
            wl_rps = (left_ticks[i + step] - left_ticks[i]) / _dt_s / ticks_per_rev * 2 * np.pi
            wr_rps = (right_ticks[i + step] - right_ticks[i]) / _dt_s / ticks_per_rev * 2 * np.pi
            wheel_speeds_rps.append([wl_rps, wr_rps])

    else:  # mocap bot records linear velocity
        for i, d in enumerate(data):
            wheel_speeds_rps.append([d[9] * distance_per_tick, -1 * d[10] * distance_per_tick])

    wheel_speeds_rps = np.array(wheel_speeds_rps)

    xs = []
    ys = []
    yaws = []
    wls = []
    wrs = []
    x = 0
    y = 0
    yaw = np.deg2rad(args.initial_yaw)
    for i, d in enumerate(wheel_speeds_rps):
        wl_rps = d[0]
        wr_rps = d[1]
        B = args.alpha * track_width_m
        T = wheel_radius_m / B * np.array([[B / 2.0, B / 2.0], [-1, 1]])
        dydt, dpdt = T @ np.array([wl_rps, wr_rps])
        x = x + np.cos(yaw) * dydt * dt_s
        y = y + np.sin(yaw) * dydt * dt_s
        yaw += dpdt * dt_s
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)
        wls.append(wl_rps)
        wrs.append(wr_rps)

    recorded_data_processing_dir = os.path.dirname(os.path.realpath(__file__))
    style = recorded_data_processing_dir + "/../phil.mplstyle"
    plt.style.use(style)

    plt.figure()
    plt.plot(np.rad2deg(yaws))
    plt.ylabel("yaw (degrees")
    plt.title("Yaws")

    plt.figure()
    plt.plot(wls, linewidth=1, label='left wheel speed')
    plt.plot(wrs, linewidth=1, label='right wheel speed')
    plt.ylabel("Angular Velocity (radians/second)")
    plt.title("Wheel Angular Speed Plots")
    plt.legend()

    plt.figure()
    plt.scatter(xs, ys, s=4, lw=0, marker='o')
    colors = cm.rainbow(np.linspace(0, 1, robot_poses.shape[0]))
    plt.scatter(mocap_xs, mocap_ys, color=colors, label='robot position', s=4, marker='.', lw=0)
    plt.xlabel("x (meters)")
    plt.ylabel("y (meters)")
    plt.axis("square")
    plt.title("Position of Robot from Encoders")

    plt.show()


if __name__ == '__main__':
    main()
