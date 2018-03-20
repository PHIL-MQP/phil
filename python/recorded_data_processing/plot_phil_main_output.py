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
    plt.show()


if __name__ == '__main__':
    main()