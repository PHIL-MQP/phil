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
    np.set_printoptions(suppress=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("data_directory", help="directory with data")

    args = parser.parse_args()

    if not os.path.exists(args.data_directory):
        print("data direction [{}] does not exist".format(args.data_directory))
        return

    data_filename = os.path.join(args.data_directory, "sensor_data.csv")
    data = np.genfromtxt(data_filename, delimiter=',')
    # 0 accel_x, (g's)
    # 1 accel_y, (g's)
    # 2 accel_z, (g's)
    # 3 gyro_x, (deg/s)
    # 4 gyro_y, (deg/s)
    # 5 gyro_z (deg/s),
    # 6 x, (m)
    # 7 y, (m)
    # 8 z, (m)
    # 9 left_encoder_rate, (m/s)
    # 10 right_encoder_rate, (m/s)
    # 11 left_input, (-1 to 1)
    # 12 right_input (-1 to 1)

    alpha = 1.0
    wheel_radius_m = 0.074
    track_width_m = 0.9
    dt_s = 0.02  # the rate of the interpolation

    N = 9
    L = 6
    M = 2
    encoder_state = np.zeros((3, 1), dtype=np.float32)  # for computing x/y/theta from encoders to go in z
    encoder_xs = []
    encoder_ys = []
    filtered_xs = []
    filtered_ys = []
    priori_xs = []
    priori_ys = []
    estimate_covariances = []
    posterior_estimate = np.zeros((N, 1), dtype=np.float32)
    # do the Q matrix thingy
    process_covariance = np.eye(N) * 1e-3
    measurement_variance = np.eye(L) * 1e-3
    estimate_covariance = np.eye(N) * 1e-3

    for row in data:
        u = np.array([[row[11]], [row[12]]], dtype=np.float32)

        T = wheel_radius_m / (alpha * track_width_m) * np.array(
            [[(alpha * track_width_m) / 2.0, (alpha * track_width_m) / 2.0], [-1, 1]])
        dydt, dpdt = T @ np.array([row[], wr])
        encoder_state[0] = encoder_state[0] + np.cos(encoder_state[2]) * dydt * dt_s
        encoder_state[1] = encoder_state[1] + np.sin(encoder_state[2]) * dydt * dt_s
        encoder_state[2] = encoder_state[2] + dpdt * dt_s
        encoder_xs.append(encoder_state[0].copy())
        encoder_ys.append(encoder_state[1].copy())

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
        B = np.array([[0.25 * cos(theta) * dt_s * dt_s, 0.25 * cos(theta) * dt_s * dt_s],
                      [0.25 * sin(theta) * dt_s * dt_s, 0.25 * sin(theta) * dt_s * dt_s],
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
                      [0, 0, 1, 0, 0, 0, 0, 0, 0],
                      ])
        gyro_theta = row[5]
        measurement = np.array(
            [[acc_x], [acc_y], [gyro_theta], [encoder_state[0]], [encoder_state[1]], [encoder_state[2]]])

        dynamics = A @ posterior_estimate
        controls = B @ u
        priori_estimate = dynamics + controls
        priori_estimate_covariance = A @ estimate_covariance @ A.T + process_covariance
        K = np.linalg.solve((C @ estimate_covariance @ C.T + measurement_variance).T, (estimate_covariance @ C.T).T).T

        K = np.zeros((N, L))

        posterior_estimate = priori_estimate + K @ (measurement - C @ priori_estimate)
        estimate_covariance = (np.eye(N) - K @ C) @ priori_estimate_covariance
        estimate_covariances.append(estimate_covariance.copy())
        ks.append(K)
        filtered_xs.append(posterior_estimate[0].copy())
        filtered_ys.append(posterior_estimate[1].copy())
        priori_xs.append(priori_estimate[0].copy())
        priori_ys.append(priori_estimate[1].copy())

    plt.figure()
    plt.plot(filtered_xs, filtered_ys, linestyle='--', label='filtered')
    plt.scatter(0, 0, marker='o', s=50, c='red')  # show starting point
    plt.legend()
    plt.axis("equal")

    plt.show()


if __name__ == '__main__':
    sys.exit(main())
