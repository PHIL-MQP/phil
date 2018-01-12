#!/usr/bin/env python3

import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def base_rotation(mean_acc_while_stationary):
    """ https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
    """
    a = mean_acc_while_stationary
    b = np.array([0, 0, 1])
    v = np.cross(a, b)
    c = np.dot(a, b)
    v_x = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + v_x + (v_x @ v_x) * (1 / (1 + c))
    return R


def angles(args):
    data = np.ndarray((args.samples, 3))
    euler_angles = np.array([args.roll, args.pitch, args.yaw])

    for i in range(args.samples):
        angles_rad = euler_angles * np.pi / 180
        cx = np.cos(angles_rad[0])
        sx = np.sin(angles_rad[0])
        cy = np.cos(angles_rad[1])
        sy = np.sin(angles_rad[1])
        cz = np.cos(angles_rad[2])
        sz = np.sin(angles_rad[2])
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

        rotated_noisy_acc_data = Rx @ Ry @ Rz @ np.array([0, 0, 1]) + np.random.normal([0] * 3, [args.noise] * 3)
        data[i] = rotated_noisy_acc_data

    means = np.mean(data, axis=0)

    # reverse solve for the original angles
    # The R can be used to transform a measurement in the IMU frame into a measurement in the base frame
    # EX: [0, 0, 1] = R @ [x0, y0, z0]
    R = base_rotation(means)
    R_inv = np.linalg.inv(R)

    # generate fake data that looks like a figure-eight
    N = 200
    ground_truth = np.ndarray((N, 9))
    for i in range(N):
        t = i / N * np.pi * 2
        x = np.cos(t)
        y = np.sin(t) * np.cos(t)
        ground_truth[i] = np.array([x, y, 0, 0, 0, 0, 0, 0, 0])

    if not args.no_plot:
        fig = plt.figure()
        ax_3d = fig.add_subplot(2, 2, 1, projection=Axes3D.name)
        acc_ax = fig.add_subplot(2, 2, 2)
        acc_ax.plot(data[:, 0], label="x")
        acc_ax.plot(data[:, 1], label="y")
        acc_ax.plot(data[:, 2], label="z")
        acc_ax.set_title("Synthetic Accelerometer Readings")
        acc_ax.set_xlabel("Time (sample #)")
        acc_ax.set_ylabel("Acceleration (g's)")
        acc_ax.legend()

        ax_3d.quiver([0, 0], [0, 0], [0, 0], [0, means[0]], [0, means[1]], [1, means[2]], length=0.05)
        ax_3d.set_xlabel("X")
        ax_3d.set_ylabel("Y")
        ax_3d.set_zlabel("Z")
        ax_3d.set_title("3d orientation vector")
        ax_3d.axis("square")

        xy_ax = fig.add_subplot(2, 2, 3)
        xy_ax.scatter(ground_truth[:, 0], ground_truth[:, 1], s=2)
        xy_ax.set_title("Synthetic Path of Travel")
        xy_ax.axis("square")
        plt.show()


def number(args):
    data = np.ndarray((args.number, args.samples, 3))
    for i in range(args.number):
        euler_angles = np.random.randint(-180, 180, 3)

        for j in range(args.samples):
            noisey_angles = euler_angles + np.random.randn(3) * args.noise
            noisey_angles_rad = noisey_angles * np.pi / 180
            # data[i, j] = noisey_angles_rad


def main():
    parser = argparse.ArgumentParser("Generates synthetic data of a stationary accelerometer at random angles")
    parser.add_argument("--noise", "-i", type=float, default=0.002, help="standard deviation of gaussian noise ("
                                                                         "degrees)")
    parser.add_argument("--samples", "-s", type=int, default=1000, help="number of samples to generate")
    parser.add_argument("--no-plot", action="store_true", help="don't show plots")
    subparsers = parser.add_subparsers()

    angle_parser = subparsers.add_parser("angle", help="specify the roll pitch and raw of the IMU relative to the base."
                                                       "We rotate around roll, pitch, then yaw")
    angle_parser.add_argument("--roll", "-r", type=float, required=True, help="roll in degrees")
    angle_parser.add_argument("--pitch", "-p", type=float, required=True, help="pitch in degrees")
    angle_parser.add_argument("--yaw", "-y", type=float, required=True, help="yaw in degrees")
    angle_parser.set_defaults(func=angles)

    number_parser = subparsers.add_parser("number", help="specify a number of random synthetic angles to simulate")
    number_parser.add_argument("number", type=int, default=1, help="the number of random angles")
    number_parser.set_defaults(func=number)

    args = parser.parse_args()
    return args.func(args)


if __name__ == '__main__':
    np.random.seed(0)
    np.set_printoptions(suppress=True)
    plt.rc('axes.formatter', useoffset=False)

    sys.exit(main())
