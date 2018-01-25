#!/usr/bin/env python3

"""
This program takes a roborio sensor data and plots position estimates for IMU, encoders, and navx-computed pose
"""

import numpy as np
import argparse
import csv
import sys
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("infile", help="mocap-data-*.csv file")

    args = parser.parse_args()

    reader = csv.reader(open(args.infile, 'r'))
    next(reader) # read header
    # 0 = accel_x
    # 1 = accel_y
    # 2 = accel_z
    # 3 = gyro_x
    # 4 = gyro_y
    # 5 = gyro_z
    # 6 = navx x
    # 7 = navx y
    # 8 = navx z
    # 9 = left_encoder_rate
    # 10 = right_encoder_rate
    # 11 = left_input
    # 12 = right_input
    # 13 = fpga time
    # 14 = navx time

    data = []
    for line in reader:
        data.append([float(d) for d in line])
    data = np.array(data)

    plt.figure()
    plt.plot(data[:, 9], label='left speed')
    plt.plot(data[:, 10], label='right speed')
    plt.ylabel("meters per second")
    plt.title("Encoder Data")
    plt.legend()

    plt.figure()
    plt.plot(data[:, 0], label='acc x')
    plt.plot(data[:, 1], label='acc y')
    plt.plot(data[:, 2], label='acc z')
    plt.title("Acc Data")
    plt.legend()

    plt.figure()
    plt.plot(data[:, 11], label='left')
    plt.plot(data[:, 12], label='right')
    plt.title("Joystick input")
    plt.legend()

    plt.figure()
    plt.plot(data[:, 3], label='gyro x')
    plt.plot(data[:, 4], label='gyro y')
    plt.plot(data[:, 5], label='gyro z')
    plt.title("Gyro Data")
    plt.legend()

    plt.figure()
    plt.scatter(data[:, 6], data[:, 7], label='navx', s=1)
    plt.title("Position")
    plt.legend()

    plt.show()


if __name__ == '__main__':
    sys.exit(main())