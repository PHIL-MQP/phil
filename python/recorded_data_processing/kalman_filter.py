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
    data_file = open(data_filename, 'r')
    reader = csv.reader(data_file)

    alpha = 1.0
    wheel_radius_m = 0.038
    track_width_m = 0.23
    acc_scale = 2  # just a guess
    dt_s = 0.02  # the rate of the interpolation

    N = 9
    L = 6
    M = 2
    encoder_state = np.zeros((3, 1), dtype=np.float32)  # for computing x/y/theta from encoders to go in z
    encoder_xs = []
    encoder_ys = []
    filtered_xs = []
    filtered_ys = []
    ks = []
    priori_xs = []
    priori_ys = []
    wls = []
    wrs = []
    estimate_covariances = []
    posterior_estimate = np.zeros((N, 1), dtype=np.float32)
    # do the Q matrix thingy
    process_covariance = np.eye(N) * 1e-3
    measurement_variance = np.eye(L) * 1e-3
    estimate_covariance = np.eye(N) * 1e-3
    next(reader)  # skip header
    first_row = next(reader)
    wls.append(float(first_row[0]))
    wrs.append(float(first_row[1]))

    just_gyro = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0]]).T
    just_acc = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0]]).T
    just_encoders = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0],
                              [1, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0, 0, 0, 0]]).T

    i = 0

    data = []
    for row in reader:
        data.append([float(d) for d in row])
    data = np.array(data)

    smoothed_ws = np.zeros((data.shape[0], 2))
    S = 50
    for idx in range(0, data.shape[0]):
        min = idx - S // 2
        max = idx + S // 2

        if min < 0:
            min = 0
        if max >= data.shape[0]:
            max = data.shape[0] - 1

        wl = data[min: max, 0]
        wr = data[min: max, 1]
        smoothed_ws[idx][0] = np.mean(wl)
        smoothed_ws[idx][1] = np.mean(wr)

    approx_acc = np.zeros((data.shape[0], 2))
    for idx in range(S, data.shape[0] - S):
        approx_acc[idx][0] = 1 / (2 * S) * (smoothed_ws[idx + S][0] - smoothed_ws[idx - S][0]) / dt_s
        approx_acc[idx][1] = 1 / (2 * S) * (smoothed_ws[idx + S][1] - smoothed_ws[idx - S][1]) / dt_s
        approx_acc[idx][0] = 0.1
        approx_acc[idx][1] = -0.3

    # sample 20 points and fit 5th order polynomial with least squares
    for idx, row in enumerate(data):
        wl = row[0]
        wr = row[1]

        wls.append(wl)
        wrs.append(wr)
        linear_al = approx_acc[idx][0] * wheel_radius_m
        linear_ar = approx_acc[idx][1] * wheel_radius_m
        u = np.array([[linear_al], [linear_ar]], dtype=np.float32)
        T = wheel_radius_m / (alpha * track_width_m) * np.array(
            [[(alpha * track_width_m) / 2.0, (alpha * track_width_m) / 2.0], [-1, 1]])
        dydt, dpdt = T @ np.array([wl, wr])
        encoder_state[0] = encoder_state[0] + np.cos(encoder_state[2]) * dydt * dt_s
        encoder_state[1] = encoder_state[1] + np.sin(encoder_state[2]) * dydt * dt_s
        encoder_state[2] = encoder_state[2] + dpdt * dt_s
        encoder_xs.append(encoder_state[0].copy())
        encoder_ys.append(encoder_state[1].copy())

        # double integrate accelerometer data
        acc_x = (row[2] - -0.0094) * acc_scale
        acc_y = (row[3] - 0.0047) * acc_scale

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

        # if i > 100:
        #     break
        # i += 1

    estimate_covariances = np.array(estimate_covariances)
    ks = np.array(ks)

    # plt.figure()
    # plt.plot(wls)
    # plt.plot(wrs)
    # plt.plot(smoothed_ws[:, 0])
    # plt.plot(smoothed_ws[:, 1])
    #
    # plt.figure()
    # plt.plot(approx_acc[:, 0])
    # plt.plot(approx_acc[:, 1])

    plt.figure()
    plt.plot(filtered_xs, filtered_ys, linestyle='--', label='filtered')
    plt.plot(encoder_xs, encoder_ys, linestyle='-', label='encoders')
    plt.scatter(0, 0, marker='o', s=50, c='red')  # show starting point
    plt.legend()
    plt.axis("equal")

    plt.show()


if __name__ == '__main__':
    sys.exit(main())
