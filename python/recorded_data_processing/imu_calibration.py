import csv
import numpy as np
from scipy.optimize import root
import matplotlib.pyplot as plt
import sys
import argparse


def acc_loss(a_i, theta_acc):
    T = np.array([[1, -theta_acc[0], theta_acc[1]], [0, 1, -theta_acc[2]], [0, 0, 1]])
    K = np.array([[theta_acc[3], 0, 0], [0, theta_acc[4], 0], [0, 0, theta_acc[5]]])
    b = np.array([[theta_acc[6], theta_acc[7], theta_acc[8]]])
    return 1 - np.linalg.norm(T @ K @ (a_i + b).T) ** 2


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


def get_static_intervals(threshold, data, t_wait, sample_per_second):
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
        print("static at the end")
        temp_pair[1] = data_array_size - window_size // 2 - 1
        static_indicators.append(temp_pair)

    return static_indicators, classifications


def optimize(intervals, static_mean_accs):
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

    residual = compute_residual(intervals, static_mean_accs, acc_params)

    return acc_params, residual


def compute_residual(intervals, static_mean_accs, acc_params):
    residual = 0
    for i in range(intervals):
        a_i = static_mean_accs[i]
        residual += acc_loss(a_i, acc_params) ** 2

    return residual


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('data_file', help='csv file of recorded IMU data')
    parser.add_argument('--t-init', type=int, default=4, help='length of init period in seconds')
    parser.add_argument('--t-wait', type=int, default=2, help='length of static intervals in seconds')
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
    for k in range(1, total_intervals + 1):
        threshold = k * sigma_init  # should be squared according to paper
        intervals, classifications = get_static_intervals(threshold, remaining_data, t_wait, samples_per_second)

        if len(intervals) != total_intervals:
            print("Skipping threshold {:0.16f}. Only found {:d}/{:d} static intervals".format(threshold, len(intervals),
                                                                                              total_intervals))
            continue

        if args.plot:
            plt.figure(figsize=(15, 10))
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

        acc_params, residual = optimize(total_intervals, static_mean_accs)

        if residual < residual_opt:
            residual_opt = residual
            acc_params_opt = acc_params
            threshold_opt = threshold
            intervals_opt = intervals

    print("Ideal accelerometer calibration parameters:")
    print(acc_params_opt)


if __name__ == '__main__':
    sys.exit(main())
