#!/usr/bin/env python3

import argparse
import sys
import matplotlib.pyplot as plt
import os

from recorded_data_processing import preprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_directory", help="directory with data")

    args = parser.parse_args()

    if not os.path.exists(args.data_directory):
        print("data direction [{}] does not exist".format(args.data_directory))
        return

    speeds, imus, metadata = preprocess.load_data(args.data_directory)

    plt.figure()
    plt.plot(speeds[:, 0], label='left speed')
    plt.plot(speeds[:, 1], label='right speed')
    plt.title("Encoder Data")
    plt.legend()

    plt.figure()
    plt.plot(imus[:, 0], label='imu x')
    plt.plot(imus[:, 1], label='imu y')
    plt.plot(imus[:, 2], label='imu z')
    plt.title("IMU Data")
    plt.legend()

    plt.figure()
    plt.plot(imus[:, 3], label='gyro x')
    plt.plot(imus[:, 4], label='gyro y')
    plt.plot(imus[:, 5], label='gyro z')
    plt.title("Gyro Data")
    plt.legend()

    plt.show()


if __name__ == '__main__':
    sys.exit(main())