import argparse
import numpy as np
import csv
import sys
import matplotlib.pyplot as plt
import os


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_directory", help="directory with data")

    args = parser.parse_args()

    if not os.path.exists(args.data_directory):
        print("data direction [{}] does not exist".format(args.data_directory))
        return

    filename = os.path.join(args.data_directory, "interpolated_data.csv")
    file = open(filename, 'r')
    reader = csv.reader(file)

    alpha = 1.0
    wheel_radius_m = 0.038
    track_width_m = 0.23
    dt_s = 0.1

    x = np.array([0, 0, 0])
    xs = []
    ys = []
    next(reader)  # skip header
    for row in reader:
        # forward kinematics
        u = np.array([float(row[0]), float(row[1])], dtype=np.float32)
        T = wheel_radius_m / (alpha * track_width_m) * np.array(
            [[(alpha * track_width_m) / 2, (alpha * track_width_m) / 2], [-1, 1]])
        dydt, dpdt = T @ u
        x[0] += np.cos(x[2]) * dydt * dt_s
        x[1] += np.sin(x[2]) * dydt * dt_s
        x[2] += dpdt * dt_s
        xs.append(x[0])
        ys.append(x[1])

        # double integrate accelerometer data

        # double integrate gyro data

    plt.plot(xs, ys, marker='.')
    plt.scatter(xs[0], ys[0], marker='o', c='red')
    plt.show()


if __name__ == '__main__':
    sys.exit(main())
