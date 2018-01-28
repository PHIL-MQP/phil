#!/usr/bin/env python3

import csv
import time
import numpy as np
from scipy.optimize import root, show_options
import matplotlib.pyplot as plt
import sys
import argparse
import math


def static_intervals(threshold, data, t_wait, sample_per_second):
    window_size = int(sample_per_second * t_wait)
    if window_size % 2 == 0:
        window_size -= 1
    data_array_size = len(data)
    classifications = np.zeros(data_array_size)
    static_indicators = []
    temp_pair = [-1, -1]
    previously_static = False
    for i in range(data_array_size - window_size):
        window_data = data[i:i + window_size]
        center = i + window_size // 2
        variance = np.sum(np.var(window_data[:, :3], axis=0) ** 2)

        # end of a static interval
        static = variance < threshold
        if not static and previously_static:
            temp_pair[1] = center
            static_indicators.append(temp_pair)
            temp_pair = [-1, -1]
        # start of a static intervals
        elif static and not previously_static:
            temp_pair[0] = center

        previously_static = static
        classifications[center] = 1 if static else 0

    if previously_static:
        temp_pair[1] = data_array_size - window_size // 2 - 1
        static_indicators.append(temp_pair)

    return static_indicators, classifications


def gyro_loss(accel_frame, previous_accel_frame, raw_gyro_readings, theta_gyro):
    """
    :param accel_frame: u_g_k
    :param previous_accel_frame: u_g_k-1
    :param raw_gyro_readings: raw gyro data between the two accelerometer frames
    :param theta_gyro: parameters for the gyro calibration
    :return:
    """

    def get_corrected_gyro_data(raw_data, theta_gyro):
        gamma_yz = theta_gyro[0]
        gamma_zy = theta_gyro[1]
        gamma_xz = theta_gyro[2]
        gamma_zx = theta_gyro[3]
        gamma_xy = theta_gyro[4]
        gamma_yx = theta_gyro[5]
        s_x = theta_gyro[6]
        s_y = theta_gyro[7]
        s_z = theta_gyro[8]
        T = np.array([[1, -gamma_yz, gamma_zy], [gamma_xz, 1, -gamma_zx], [-gamma_xy, gamma_yx, 1]])
        K = np.array([[s_x, 0, 0], [0, s_y, 0], [0, 0, s_z]])

        corrected_data = T @ K @ np.expand_dims(raw_data, 2)

        return corrected_data

    def rotation_matrix(gyro_matrix):
        theta_x, theta_y, theta_z = gyro_matrix[0], gyro_matrix[1], gyro_matrix[2]

        cx = math.cos(theta_x)
        sx = math.sin(theta_x)
        cy = math.cos(theta_y)
        sy = math.sin(theta_y)
        cz = math.cos(theta_z)
        sz = math.sin(theta_z)
        rotation_matrix_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        rotation_matrix_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        rotation_matrix_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

        return rotation_matrix_x @ rotation_matrix_y @ rotation_matrix_z

    def integrate_gyro(initial_acc, gyro_data, dt):
        conversion = np.pi / 180
        for g in gyro_data:
            initial_acc = rotation_matrix(g * dt * conversion) @ initial_acc
        return initial_acc

    def expected_acc_frame(acc_frame, raw_gyro_data, theta_gyro, dt):
        corrected_gyro_data = get_corrected_gyro_data(raw_gyro_data, theta_gyro)
        expected_final_acc_frames = integrate_gyro(acc_frame, corrected_gyro_data, dt)

        return expected_final_acc_frames

    ugk = expected_acc_frame(previous_accel_frame, raw_gyro_readings, theta_gyro, 0.01)
    ugk = ugk / np.linalg.norm(ugk)

    uak = accel_frame / np.linalg.norm(accel_frame)
    return np.linalg.norm(uak - ugk)


def acc_loss(a_i, theta_acc):
    T = np.array([[1, -theta_acc[0], theta_acc[1]], [0, 1, -theta_acc[2]], [0, 0, 1]])
    K = np.array([[theta_acc[3], 0, 0], [0, theta_acc[4], 0], [0, 0, theta_acc[5]]])
    b = np.array([[theta_acc[6], theta_acc[7], theta_acc[8]]])
    return 1 - np.linalg.norm(T @ K @ (a_i + b).T) ** 2


def gyro_loss_derivative(accel_frame, previous_accel_frame, raw_gyro_readings, theta_gyro, h=0.1):
    def func(nodes_):
        return gyro_loss(accel_frame, previous_accel_frame, raw_gyro_readings, nodes_)

    n = theta_gyro.shape[1]
    partials = np.ndarray(n)
    for partial_idx in range(n):
        hs = np.array([-2 * h, -h, h, 2 * h])
        nodes = np.repeat(theta_gyro, 4, axis=0)
        nodes[:, partial_idx] += hs
        five_point_approx = 1 / (12 * h) * (func(nodes[0]) - 8 * func(nodes[1]) + 8 * func(nodes[2]) - func(nodes[3]))
        partials[partial_idx] = five_point_approx
    return partials


def acc_loss_derivative(a_i, theta_acc, h=0.1):
    def func(nodes_):
        return acc_loss(a_i, nodes_)

    n = theta_acc.shape[1]
    partials = np.ndarray(n)
    for partial_idx in range(n):
        hs = np.array([-2 * h, -h, h, 2 * h])
        nodes = np.repeat(theta_acc, 4, axis=0)
        nodes[:, partial_idx] += hs
        five_point_approx = 1 / (12 * h) * (func(nodes[0]) - 8 * func(nodes[1]) + 8 * func(nodes[2]) - func(nodes[3]))
        partials[partial_idx] = five_point_approx
    return partials


def gyro_optimize(num_intervals, static_mean_accs, all_raw_gyro_readings, intervals):
    """
    :param num_intervals: number of static intervals (M)
    :param static_mean_accs: calibrated average accelerometer readings during each static interval
    :param all_raw_gyro_readings: all gyro readings after init period [?X3]
    :param intervals: the range of the detected static intervals [num_intervalsx2]
    :return:
    """
    theta_gyro = np.array([0, 0, 0, 0, 0, 0, 1, 1, 1], dtype=np.float64)

    # optimization of accelerometer parameters
    def gyro_error_and_jacobian(params):
        # for each static interval
        # t0 = time.time()
        f = np.zeros(num_intervals)
        # df = np.zeros((num_intervals, 9))
        for i in range(1, num_intervals):
            previous_accel_frame = static_mean_accs[i - 1]
            accel_frame = static_mean_accs[i]
            raw_gyro_readings = all_raw_gyro_readings[intervals[i - 1][1]:intervals[i][0]]
            f[i] = gyro_loss(accel_frame, previous_accel_frame, raw_gyro_readings, params)
            # df[i] = gyro_loss_derivative(accel_frame, previous_accel_frame, raw_gyro_readings,
            #                              np.expand_dims(params, 0))
        # return f, df
        return f

    sol = root(gyro_error_and_jacobian, theta_gyro, jac=False, method='lm', options={'maxiter': 1000})
    gyro_params = sol.x

    if not sol.success:
        print("Optimizer Failed: ", sol.message, gyro_params)
    else:
        print("Gyro LM Converged! in {:d} iterations".format(sol.nfev))

    residual = compute_residual(num_intervals, static_mean_accs, gyro_params)

    return gyro_params, residual


def acc_optimize(intervals, static_mean_accs):
    theta_acc = np.array([0, 0, 0, 1, 1, 1, 0, 0, 0], dtype=np.float64)

    # optimization of accelerometer parameters
    def accelerometer_error_and_jacobian(params):
        # for each static interval
        f = np.ndarray((intervals))
        df = np.ndarray((intervals, 9))
        for i in range(intervals):
            a_i = static_mean_accs[i]
            f[i] = acc_loss(a_i, params)
            df[i] = acc_loss_derivative(a_i, np.expand_dims(params, 0))
        return f, df

    sol = root(accelerometer_error_and_jacobian, theta_acc, jac=True, method='lm', options={'maxiter': 10})
    acc_params = sol.x

    if not sol.success:
        print("Optimizer Failed: ", sol.message, acc_params)
    else:
        print("Accelerometer Converged! in {:d} iterations".format(sol.nfev))

    residual = compute_residual(intervals, static_mean_accs, acc_params)

    return acc_params, residual


def compute_residual(intervals, static_mean_accs, acc_params):
    residual = 0
    for i in range(intervals):
        a_i = static_mean_accs[i]
        residual += acc_loss(a_i, acc_params) ** 2

    return residual


def double_integrate_acc(acc_data, dt_s, K, T, b):
    x = 0
    y = 0
    vx = 0
    vy = 0
    xs = []
    ys = []
    for a_s in acc_data:
        a_o = T @ K @ (a_s + b).T
        ax = a_o[0][0]
        ay = a_o[1][0]

        vx += ax * dt_s
        vy += ay * dt_s
        x += vx * dt_s + 0.5 * ax * dt_s ** 2
        y += vy * dt_s + 0.5 * ay * dt_s ** 2
        xs.append(x)
        ys.append(y)

    return xs, ys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('data_file', help='csv file of recorded IMU data')
    parser.add_argument('--t-init', type=float, default=4, help='length of init period in seconds')
    parser.add_argument('--t-wait', type=float, default=2, help='length of static intervals in seconds')
    parser.add_argument('--intervals', type=int, default=19,
                        help='number of static intervals (not including init period)')
    parser.add_argument('--plot', action="store_true", help='show a plot of the generated data')

    args = parser.parse_args()

    reader = csv.reader(open(args.data_file, 'r'))
    next(reader)  # skip header
    data = []
    for row in reader:
        data.append([float(x) for x in row])
    data = np.array(data)

    # Process the initial calibration period (T init)
    t_wait = args.t_wait
    t_init = args.t_init
    samples_per_second = 100
    t_init_idx = int(t_init * samples_per_second)
    init_data = data[:t_init_idx]
    gyro_biases = np.mean(init_data, axis=0)[3:6]  # line 1
    data = data - np.array([0, 0, 0, gyro_biases[0], gyro_biases[1], gyro_biases[2], 0])  # line 2
    remaining_data = data[t_init_idx:]
    sigma_init = np.linalg.norm(np.var(init_data[:, :3], axis=0))  # line 4

    # Accelerometer calibration
    total_intervals = args.intervals
    residual_opt = float("inf")
    for k in range(1, total_intervals + 1):  # line 5
        # for k in range(1, 5):  # line 5
        threshold = k * sigma_init ** 2  # line 6 -- should be squared according to paper

        intervals, classifications = static_intervals(threshold, remaining_data, t_wait, samples_per_second)  # line 7

        if len(intervals) != total_intervals:
            print("Skipping threshold {:0.16f}. Found {:d}/{:d} static intervals".format(threshold, len(intervals),
                                                                                         total_intervals))
            continue
        else:
            print("Threshold {:0.16f} found all {:d} intervals".format(threshold, len(intervals)))

        if args.plot:
            plt.title("Static Classifier for threshold %0.16f" % threshold)
            plt.plot(classifications, c='k', label="static")
            plt.plot(remaining_data[:, 0], label='accel x', alpha=0.4, c='m')
            plt.plot(remaining_data[:, 1], label='accel y', alpha=0.4, c='b')
            plt.plot(remaining_data[:, 2], label='accel z', alpha=0.4, c='y')
            plt.legend(bbox_to_anchor=(1.1, 1))
            plt.show()

        # compute the average
        static_mean_accs = np.ndarray((total_intervals, 3))
        for i in range(total_intervals):
            static_interval = intervals[i]
            static_mean_accs[i] = np.mean(remaining_data[static_interval], axis=0)[0:3]

        acc_params, residual = acc_optimize(total_intervals, static_mean_accs)  # line 8

        if residual < residual_opt:  # lines 9-12
            residual_opt = residual
            acc_params_opt = acc_params
            threshold_opt = threshold
            intervals_opt = intervals
            classifications_opt = classifications
            static_mean_accs_opt = static_mean_accs

    plt.title("Static Classifier for threshold %0.16f" % threshold_opt)
    plt.plot(classifications_opt, c='k', label="static")
    plt.plot(remaining_data[:, 0], label='accel x', alpha=0.4, c='m')
    plt.plot(remaining_data[:, 1], label='accel y', alpha=0.4, c='b')
    plt.plot(remaining_data[:, 2], label='accel z', alpha=0.4, c='y')
    plt.legend(bbox_to_anchor=(1.1, 1))
    plt.show()

    gyro_params_opt, residual = gyro_optimize(total_intervals, static_mean_accs_opt, remaining_data[:, 3:6],
                                              intervals_opt)

    print("Ideal accelerometer calibration parameters:")
    print(acc_params_opt)
    print("Ideal gyro calibration parameters:")
    print(gyro_params_opt)


if __name__ == '__main__':
    sys.exit(main())
