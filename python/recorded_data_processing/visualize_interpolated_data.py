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

    acc_state = np.zeros((1, 9), dtype=np.float32)
    acc_xs = []
    acc_ys = []
    encoder_state = np.array([0, 0, 0], dtype=np.float32)
    encoder_xs = []
    encoder_ys = []
    filtered_xs = []
    filtered_ys = []
    wls = []
    wrs = []
    posterior_estimate = np.zeros((1, 9), dtype=np.float32)
    process_noise_variance = 0.01
    estimate_covariance = np.array([0.01] * 9)
    measurement_variance = 0.1
    next(reader)  # skip header
    for row in reader:
        # this is wrong, don't use measured speed as control inputs
        # we didn't measure actual control inputs on turtlebot
        # so we just set a* to 0 because we suck
        wl = float(row[0])
        wr = float(row[1])
        wls.append(wl)
        wrs.append(wr)
        al = 0
        ar = 0
        w = np.array([wl, wr, al, ar], dtype=np.float32)
        B = alpha * track_width_m
        T = wheel_radius_m / B * np.array([[B / 2.0, B / 2.0], [-1, 1]])
        dydt, dpdt = T @ np.array([wl, wr])
        encoder_state[0] += np.cos(encoder_state[2]) * dydt * dt_s
        encoder_state[1] += np.sin(encoder_state[2]) * dydt * dt_s
        encoder_state[2] += dpdt * dt_s
        encoder_xs.append(encoder_state[0])
        encoder_ys.append(encoder_state[1])

        # double integrate accelerometer data
        acc_x = float(row[2]) * acc_scale
        acc_y = float(row[3]) * acc_scale
        acc_state[0, 0] += dt_s * acc_state[0, 3] + 0.5 * pow(dt_s, 2) * acc_x
        acc_state[0, 1] += dt_s * acc_state[0, 4] + 0.5 * pow(dt_s, 2) * acc_y
        acc_state[0, 3] += dt_s * acc_x
        acc_state[0, 4] += dt_s * acc_y
        acc_state[0, 6] += acc_x
        acc_state[0, 7] += acc_y
        acc_xs.append(acc_state[0, 0])
        acc_ys.append(acc_state[0, 1])

        # double integrate gyro data

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
        theta = posterior_estimate[0, 2]
        W = track_width_m
        R = wheel_radius_m
        B = np.array([[0.5 * cos(theta) * dt_s, 0.5 * cos(theta) * dt_s, 0.25 * cos(theta) * dt_s * dt_s,
                       0.25 * cos(theta) * dt_s * dt_s],
                      [0.5 * sin(theta) * dt_s, 0.5 * sin(theta) * dt_s, 0.25 * sin(theta) * dt_s * dt_s,
                       0.25 * sin(theta) * dt_s * dt_s],
                      [R * dt_s / W, -R * dt_s / W, R * dt_s * dt_s / (2 * W), -R * dt_s * dt_s / (2 * W)],
                      [0, 0, 0.5 * cos(theta) * dt_s, 0.5 * cos(theta) * dt_s],
                      [0, 0, 0.5 * sin(theta) * dt_s, 0.5 * sin(theta) * dt_s],
                      [0, 0, R * dt_s / W, -R * dt_s / W],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]
                      ])
        u = w
        measurement = np.array([acc_state])

        process_noise = np.random.normal(0, process_noise_variance, 9)
        priori_estimate = A @ posterior_estimate.T + B @ u
        priori_estimate_covariance = A @ estimate_covariance @ A.T + process_noise_variance
        K = estimate_covariance / (estimate_covariance + measurement_variance)
        posterior_estimate = priori_estimate + K @ (measurement - priori_estimate)
        estimate_covariance = (np.eye(9) - K) @ priori_estimate_covariance
        filtered_xs.append(posterior_estimate[0, 0])
        filtered_ys.append(posterior_estimate[0, 1])

    plt.plot(filtered_xs, filtered_ys, marker='x')
    plt.plot(encoder_xs, encoder_ys, marker='.')
    plt.plot(acc_xs, acc_ys, marker='.')
    plt.scatter(0, 0, marker='o', c='red')  # show starting point
    plt.axis("equal")

    plt.show()


if __name__ == '__main__':
    sys.exit(main())
