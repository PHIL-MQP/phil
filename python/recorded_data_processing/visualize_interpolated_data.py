#!/usr/bin/env python3

import argparse
import json

import numpy as np
from math import cos, sin
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

    data_filename = os.path.join(args.data_directory, "interpolated_data.csv")
    metadata_filename = os.path.join(args.data_directory, "metadata.json")
    metadata = json.load(open(metadata_filename, 'r'))
    data_file = open(data_filename, 'r')
    reader = csv.reader(data_file)

    alpha = 1.0
    wheel_radius_m = 0.038
    track_width_m = 0.23
    acc_scale = 1
    dt_s = 0.1  # the rate of the interpolation

    N = 9
    L = 6
    M = 2
    acc_state = np.zeros((N, 1), dtype=np.float32)
    acc_xs = []
    acc_ys = []
    encoder_state = np.zeros((3, 1), dtype=np.float32)  # for computing x/y/theta from encoders to go in z
    encoder_xs = []
    encoder_ys = []
    filtered_xs = []
    filtered_ys = []
    dynamics_xs = []
    dynamics_ys = []
    wls = []
    wrs = []
    estimate_covariances = []
    posterior_estimate = np.zeros((N, 1), dtype=np.float32)
    W = 0.01  # process variance
    # do the Q matrix thingy
    process_covariance = np.diag([.01, .01, .01, .01, .01, .01, .01, .01, .01])
    measurement_variance = np.zeros((L, L))
    estimate_covariance = np.zeros((N, N))
    next(reader)  # skip header
    first_row = next(reader)
    wls.append(float(first_row[0]))
    wrs.append(float(first_row[1]))
    for row in reader:
        # this is wrong, don't use measured speed as control inputs
        # we didn't measure actual control inputs on turtlebot
        # so we just set a* to 0 because we suck
        wl = float(row[0])
        wr = float(row[1])
        al = (wl - wls[-1]) / dt_s
        ar = (wr - wrs[-1]) / dt_s
        wls.append(wl)
        wrs.append(wr)
        u = np.array([[al], [ar]], dtype=np.float32)
        B = alpha * track_width_m
        T = wheel_radius_m / B * np.array([[B / 2.0, B / 2.0], [-1, 1]])
        dydt, dpdt = T @ np.array([wl, wr])
        encoder_state[0] = posterior_estimate[0] + np.cos(posterior_estimate[2]) * dydt * dt_s
        encoder_state[1] = posterior_estimate[1] + np.sin(posterior_estimate[2]) * dydt * dt_s
        encoder_state[2] = posterior_estimate[2] + dpdt * dt_s
        encoder_xs.append(encoder_state[0])
        encoder_ys.append(encoder_state[1])

        # double integrate accelerometer data
        acc_x = float(row[2]) * acc_scale
        acc_y = float(row[3]) * acc_scale
        acc_state[0] += dt_s * posterior_estimate[3] + 0.5 * pow(dt_s, 2) * acc_x
        acc_state[1] += dt_s * posterior_estimate[4] + 0.5 * pow(dt_s, 2) * acc_y
        acc_state[3] += dt_s * acc_x
        acc_state[4] += dt_s * acc_y
        acc_state[6] += acc_x
        acc_state[7] += acc_y
        acc_xs.append(acc_state[0])
        acc_ys.append(acc_state[1])

        # kalman filter
        A = np.array([[1, 0, 0, dt_s, 0, 0, 0.5 * dt_s * dt_s, 0, 0],
                      [0, 1, 0, 0, dt_s, 0, 0, 0.5 * dt_s * dt_s, 0],
                      [0, 0, 1, 0, 0, dt_s, 0, 0, 0.5 * dt_s * dt_s],
                      [0, 0, 0, 1, 0, 0, dt_s, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, dt_s, 0],
                      [0, 0, 0, 0, 0, 1, 0, 0, dt_s],
                      [0, 0, 0, 0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 1],
                      ])
        theta = posterior_estimate[2]
        W = track_width_m
        R = wheel_radius_m
        B = np.array([[0.25 * cos(theta) * dt_s * dt_s,
                       0.25 * cos(theta) * dt_s * dt_s],
                      [0.25 * sin(theta) * dt_s * dt_s,
                       0.25 * sin(theta) * dt_s * dt_s],
                      [R * dt_s * dt_s / (2 * W), -R * dt_s * dt_s / (2 * W)],
                      [0.5 * cos(theta) * dt_s, 0.5 * cos(theta) * dt_s],
                      [0.5 * sin(theta) * dt_s, 0.5 * sin(theta) * dt_s],
                      [R * dt_s / W, -R * dt_s / W],
                      [0, 0],
                      [0, 0],
                      [0, 0]
                      ])
        C = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1, 0, 0, 0],
                      # [1, 0, 0, 0, 0, 0, 0, 0, 0],
                      # [0, 1, 0, 0, 0, 0, 0, 0, 0],
                      # [0, 0, 1, 0, 0, 0, 0, 0, 0],
                      [1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0, 0, 0, 0]])
        gyro_theta = float(row[5])
        measurement = np.array(
            [[acc_x], [acc_y], [gyro_theta], [encoder_state[0]], [encoder_state[1]], [encoder_state[2]]])

        priori_estimate = (A @ posterior_estimate + B @ u)
        priori_estimate_covariance = A @ estimate_covariance @ A.T + process_covariance
        thingy = C @ estimate_covariance @ C.T + measurement_variance
        K = np.linalg.solve(thingy.T, (estimate_covariance @ C.T).T)
        posterior_estimate = priori_estimate + K @ (measurement - C @ priori_estimate)
        estimate_covariance = (np.eye(N) - K) @ priori_estimate_covariance
        estimate_covariances.append(estimate_covariance)
        filtered_xs.append(posterior_estimate[0, 0])
        filtered_ys.append(posterior_estimate[0, 1])
        dynamics_xs.append(priori_estimate[0, 0])
        dynamics_ys.append(priori_estimate[0, 1])

    plt.plot(estimate_covariances[:][0], label='estimate covariance of X')
    plt.plot(estimate_covariances[:][1], label='estimate covariance of Y')
    plt.legend()

    plt.figure()
    plt.plot(filtered_xs, filtered_ys, linestyle='--', label='filtered')
    plt.plot(dynamics_xs, dynamics_ys, label='priori')
    plt.plot(encoder_xs, encoder_ys, marker='.', label='encoders')
    plt.plot(acc_xs, acc_ys, marker='.', label='accel')
    plt.scatter(0, 0, marker='o', c='red')  # show starting point
    plt.legend()
    plt.axis("equal")

    plt.show()


if __name__ == '__main__':
    sys.exit(main())
