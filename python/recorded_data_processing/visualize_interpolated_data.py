#!/usr/bin/env python3

import argparse
import json

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

    acc_state = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
    acc_xs = []
    acc_ys = []
    encoder_state = np.array([0, 0, 0], dtype=np.float32)
    encoder_xs = []
    encoder_ys = []
    wls = []
    wrs = []
    posterior_estimate = np.zeros((6, 1), dtype=np.float32)
    process_noise_variance = 0.01
    estimate_covariance = np.array([0.01]*6)
    measurement_variance = 0.1
    next(reader)  # skip header
    for row in reader:
        # linearized dynamics, which represents Ax + Bu
        # this part calculated the expected next state given our model of robot moves given the current state
        # our x matrix contains x, y, theta positions, velocities, accelerations, and several error terms
        wl = float(row[0])
        wr = float(row[1])
        wls.append(wl)
        wrs.append(wr)
        w = np.array([wl, wr], dtype=np.float32)
        B = alpha * track_width_m
        T = wheel_radius_m / B * np.array([[B / 2.0, B / 2.0], [-1, 1]])
        dydt, dpdt = T @ w
        encoder_state[0] += np.cos(encoder_state[2]) * dydt * dt_s
        encoder_state[1] += np.sin(encoder_state[2]) * dydt * dt_s
        encoder_state[2] += dpdt * dt_s
        encoder_xs.append(encoder_state[0])
        encoder_ys.append(encoder_state[1])

        # double integrate accelerometer data
        acc_x = float(row[2]) * acc_scale
        acc_y = float(row[3]) * acc_scale
        acc_state[3] += dt_s * acc_x
        acc_state[4] += dt_s * acc_y
        acc_state[0] += dt_s * acc_state[3] + 0.5*pow(dt_s, 2) * acc_x
        acc_state[1] += dt_s * acc_state[4] + 0.5*pow(dt_s, 2) * acc_y
        acc_xs.append(acc_state[0])
        acc_ys.append(acc_state[1])

        # double integrate gyro data

        # kalman filter
        A = np.array([[1, 0, 0, dt_s, 0, 0],
                      [0, 1, 0, 0, dt_s, 0],
                      [0, 0, 1, 0, 0, dt_s],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      ])
        B = np.array([[0, 0],
                      [0, 0],
                      [dt_s / (wheel_radius_m - track_width_m/2), 0],
                      [0.5*np.cos(posterior_estimate[2]), 0.5*np.cos(posterior_estimate[2])],
                      [0.5*np.sin(posterior_estimate[2]), 0.5*np.sin(posterior_estimate[2])],
                      [1/track_width_m, -1/track_width_m]])
        u = w
        measurement = np.array([acc_state])

        process_noise = np.random.normal(0, process_noise_variance, 6)
        priori_estimate = A@posterior_estimate + B@u
        print(A.shape, estimate_covariance.shape)
        priori_estimate_covariance = A@estimate_covariance@A.T + process_noise_variance
        K = estimate_covariance/(estimate_covariance+measurement_variance)
        posterior_estimate = priori_estimate + K@(measurement - priori_estimate)
        estimate_covariance = (np.eye(6) - K) @ priori_estimate_covariance

    # plt.plot(encoder_xs, encoder_ys, marker='.')
    plt.plot(acc_xs, acc_ys, marker='.')
    plt.scatter(0, 0, marker='o', c='red')  # show starting point
    plt.axis("equal")

    plt.show()


if __name__ == '__main__':
    sys.exit(main())
