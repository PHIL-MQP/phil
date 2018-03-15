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
        track_width_m = 0.1
        wheel_radius_m = 0.04
        dt_s = 0.01
    else:
        track_width_m = 0.9
        wheel_radius_m = 0.074
        dt_s = 0.05
        distance_per_tick = 0.000357

    print(args)

    data = np.genfromtxt(args.csv_file, delimiter=',', skip_header=True)

    xs = []
    ys = []
    yaws = []
    wls = []
    wrs = []
    x = 0
    y = 0
    yaw = 0
    for i, d in enumerate(data):
        # these must be angular velocities in radians per second
        if args.turtlebot:  # turtlebot gets ticks. FIXME: how many ticks per revolution?
            wl_rps = (data[i+1, 0] - d[0]) / dt_s / 2**16 * 2 * np.pi
            wr_rps = (data[i+1, 1] - d[1]) / dt_s / 2**16 * 2 * np.pi
        else:  # mocap bot records linear velocity
            wl_rps = d[10] * distance_per_tick
            wr_rps = d[11] * distance_per_tick

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

    colors = cm.rainbow(np.linspace(0, 1, data.shape[0]))
    plt.scatter(xs, ys, color=colors, label='robot position')
    plt.xlabel("x (meters)")
    plt.ylabel("y (meters)")
    plt.axis("square")

    plt.figure()
    plt.plot(wls, label='left wheel speed')
    plt.plot(wrs, label='right wheel speed')
    plt.title("Wheel Angular Speed Plots")
    plt.ylabel("Angular Velocity (radians/second)")

    plt.show()


if __name__ == '__main__':
    main()
