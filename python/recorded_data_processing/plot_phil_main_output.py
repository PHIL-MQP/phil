import os
import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser("plot the output of phil_main")
    parser.add_argument("infile", help="the file you piped phil_main to")

    args = parser.parse_args()

    data = np.genfromtxt(args.infile, delimiter=", ", skip_footer=1)
    xs = data[:, 0]
    ys = data[:, 1]

    recorded_data_processing_dir = os.path.dirname(os.path.realpath(__file__))
    style = recorded_data_processing_dir + "/../phil.mplstyle"
    plt.style.use(style)
    plt.plot(xs, ys)
    plt.title("position of robot")
    plt.xlabel("u (meters)")
    plt.xlabel("x (meters)")
    plt.axis('square')

    plt.figure()
    plt.plot(data[:, 2])
    plt.ylabel("angle")
    plt.xlabel("time")

    plt.figure()
    plt.plot(data[:, 3], label='vx')
    plt.plot(data[:, 4], label='vy')
    plt.legend()
    plt.ylabel("velocity")
    plt.xlabel("time")

    # plt.figure()
    # plt.plot(data[:, 5], label='cov x')
    # plt.plot(data[:, 6], label='cov y')
    # plt.plot(data[:, 7], label='cov z')
    # plt.legend()
    # plt.ylabel("variance")
    # plt.xlabel("time")

    plt.show()


if __name__ == '__main__':
    main()
