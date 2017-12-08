import csv
import numpy as np
from scipy.optimize import root
import matplotlib.pyplot as plt
import sys
import argparse


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
    pass


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
        f = np.ndarray((num_intervals))
        df = np.ndarray((num_intervals, 9))
        for i in range(1, num_intervals):
            previous_accel_frame = static_mean_accs[i-1]
            accel_frame = static_mean_accs[i]
            raw_gyro_readings = all_raw_gyro_readings[intervals[i-1][1]:intervals[i][0]]
            f[i] = gyro_loss(accel_frame, previous_accel_frame, raw_gyro_readings, params)
            df[i] = gyro_loss_derivative(accel_frame, previous_accel_frame, raw_gyro_readings, params)
        return f, df

    sol = root(gyro_error_and_jacobian, theta_gyro, jac=True, method='lm')
    gyro_params = sol.x

    if not sol.success:
        print("Optimizer Failed: ", sol.message, gyro_params)
    # else:
    #     print(gyro_params)

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

    sol = root(accelerometer_error_and_jacobian, theta_acc, jac=True, method='lm')
    acc_params = sol.x

    if not sol.success:
        print("Optimizer Failed: ", sol.message, acc_params)
    else:
        print("LM Converged!")

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
    # for k in range(1, 2):  # line 5
        threshold = k * sigma_init**2  # line 6 -- should be squared according to paper
        intervals, classifications = static_intervals(threshold, remaining_data, t_wait, samples_per_second)  # line 7

        if len(intervals) != total_intervals:
            print("Skipping threshold {:0.16f}. Found {:d}/{:d} static intervals".format(threshold, len(intervals),
                                                                                         total_intervals))
            continue

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
            static_mean_accs_opt = static_mean_accs

    # gyro_params, residual = gyro_optimize(total_intervals, static_mean_accs_opt, remaining_data[:,3:6], intervals_opt)

    print("Ideal accelerometer calibration parameters:")
    print(acc_params_opt)
    print(residual_opt)
    # print("Ideal gyro calibration parameters:")
    # print(gyro_params)


if __name__ == '__main__':
    sys.exit(main())
