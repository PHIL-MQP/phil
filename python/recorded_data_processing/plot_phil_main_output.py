#!/usr/bin/env python3
import os
import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser("plot the output of phil_main")
    parser.add_argument("infile", help="the file you piped phil_main to")
    parser.add_argument("--start", help="plot form start to end", type=int, default=0)
    parser.add_argument("--end", help="plot form start to end", type=int, default=-1)

    args = parser.parse_args()

    start = args.start
    end = args.end

    data = np.genfromtxt(args.infile, delimiter=", ", skip_footer=1)
    yaws = data[:, 2]

    recorded_data_processing_dir = os.path.dirname(os.path.realpath(__file__))
    style = recorded_data_processing_dir + "/../phil.mplstyle"

    plt.style.use(style)
    plt.plot(data[start:end, 0], data[start:end, 1])
    skip = 10
    plt.quiver(data[start:end:skip, 0], data[start:end:skip, 1], np.cos(yaws[start:end:skip]), np.sin(yaws[start:end:skip]), width=0.01)
    plt.title("position of robot")
    plt.xlabel("y (meters)")
    plt.xlabel("x (meters)")
    plt.axis('square')

    plt.figure()
    plt.plot(yaws)
    plt.ylabel("yaw angle (radians)")
    plt.xlabel("samples")

    plt.figure()
    plt.plot(data[:, 3], label='vx')
    plt.plot(data[:, 4], label='vy')
    plt.legend()
    plt.ylabel("velocity")
    plt.xlabel("samples")

    plt.figure()
    plt.plot(data[:, 6], label='ax')
    plt.plot(data[:, 7], label='ay')
    plt.legend()
    plt.ylabel("acceleration")
    plt.xlabel("time")

    plt.figure()
    plt.plot(data[:, 9], label='cov x')
    plt.plot(data[:, 10], label='cov y')
    plt.plot(data[:, 11], label='cov yaw')
    plt.legend()
    plt.ylabel("variance")
    plt.xlabel("time")

    plt.show()


if __name__ == '__main__':
    main()
