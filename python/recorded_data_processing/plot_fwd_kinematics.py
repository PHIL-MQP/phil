#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm


def main():
    parser = argparse.ArgumentParser("run forward kinematics on encoder data from CSV")
    parser.add_argument('csv_file', help='csv file with some columns of encoder data')
    robots = parser.add_mutually_exclusive_group(required=True)
    robots.add_argument('--turtlebot', action='store_true')
    robots.add_argument('--mocapbot', action='store_true')
    parser.add_argument('--alpha', help='slip value, must be >=1', default=1, type=float)

    args = parser.parse_args()

    if args.turtlebot:
        track_width_m = 0.23
        wheel_radius_m = 0.035
        step = 6
        dt_s = 0.023 * step
        ticks_per_rev = 2578.33
    else:
        track_width_m = 0.9
        wheel_radius_m = 0.074
        dt_s = 0.05
        distance_per_tick = 0.000357

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
            wheel_speeds_rps.append([d[10] * distance_per_tick, d[11] * distance_per_tick])

    wheel_speeds_rps = np.array(wheel_speeds_rps)

    xs = []
    ys = []
    yaws = []
    wls = []
    wrs = []
    x = 0
    y = 0
    yaw = 0
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

    plt.figure()
    plt.plot(wls, linewidth=1, label='left wheel speed')
    plt.plot(wrs, linewidth=1, label='right wheel speed')
    plt.title("Wheel Angular Speed Plots")
    plt.ylabel("Angular Velocity (radians/second)")

    plt.figure()
    colors = cm.rainbow(np.linspace(0, 1, wheel_speeds_rps.shape[0]))
    plt.plot(xs, ys, linewidth=1)
    plt.scatter(xs, ys, color=colors, label='robot position', s=8)
    plt.xlabel("x (meters)")
    plt.ylabel("y (meters)")
    plt.axis("square")

    plt.show()


if __name__ == '__main__':
    main()
