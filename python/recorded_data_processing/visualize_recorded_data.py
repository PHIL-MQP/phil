import argparse
import sys
import numpy as np
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
    plt.title("Raw Sensor Data")

    plt.show()


if __name__ == '__main__':
    sys.exit(main())