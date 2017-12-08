import argparse
import numpy as np
import sys
from imu_calibration import acc_optimize, compute_residual, double_integrate_acc
import matplotlib.pyplot as plt
import csv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("outfile", help="output file (csv)")
    parser.add_argument('--twait', type=int, default=4, help='the length of each interval')
    parser.add_argument('--intervals', type=int, default=20, help='number of intervals')
    parser.add_argument('--noise', type=float, default=1e-3, help='variance of the gaussian noise added')
    parser.add_argument('--seed', type=int, default=1, help='seed numpy random generator')
    parser.add_argument('--plot', action="store_true", help='show a plot of the generated data')
    args = parser.parse_args()

    np.set_printoptions(suppress=True)  # no scientific notation

    writer = csv.writer(open(args.outfile, 'w'))

    np.random.seed(args.seed)  # for reproducibility
    acc_params = [0.1, .01, -.01, 1, 0.95, 1.08, -0.01, 0.03, 0.05]
    gyro_params = [0.1, .01, -.01, 1, 0.95, 1.08, -0.01, 0.03, 0.05]

    # generate fake sample of sitting flat on table
    num_samples = args.twait * 100
    data = []
    fake_mean_accs = np.ndarray((args.intervals, 3))
    for interval in range(args.intervals):
        random_attitude = np.random.randn(3)
        random_attitude[2] += 6
        norm = np.linalg.norm(random_attitude)
        true_a = random_attitude / norm
        true_g = np.array([0, 0, 0], dtype=np.float64)

        for i in range(num_samples):

            noise = np.random.normal(0, args.noise, 3)

            # skew this data such that our above params would correctly transform it
            Ta = np.array([[1, -acc_params[0], acc_params[1]], [0, 1, -acc_params[2]], [0, 0, 1]])
            Ka = np.array([[acc_params[3], 0, 0], [0, acc_params[4], 0], [0, 0, acc_params[5]]])
            ba = np.array([[acc_params[6]], [acc_params[7]], [acc_params[8]]])
            skewed_a = np.linalg.inv(Ka) @ np.linalg.inv(Ta) @ true_a - ba.T + noise
            Tg = np.array([[1, -gyro_params[0], gyro_params[1]], [0, 1, -gyro_params[2]], [0, 0, 1]])
            Kg = np.array([[gyro_params[3], 0, 0], [0, gyro_params[4], 0], [0, 0, gyro_params[5]]])
            bg = np.array([[gyro_params[6]], [gyro_params[7]], [gyro_params[8]]])
            skewed_g = np.linalg.inv(Kg) @ np.linalg.inv(Tg) @ true_g - bg.T + noise

            row = [skewed_a[0][0], skewed_a[0][1], skewed_a[0][2], skewed_g[0][0], skewed_g[0][1], skewed_g[0][2], 0]
            data.append(row)
            writer.writerow(row)
        fake_mean_accs[interval] = np.mean(data[interval*num_samples:(interval+1)*num_samples], axis=0)[0:3]

    estimated_acc_params, residual = acc_optimize(args.intervals, fake_mean_accs)
    print("true parameters:\n", acc_params)
    print("residual of those params: ", compute_residual(args.intervals, fake_mean_accs, acc_params))
    print("paramaters found:\n", estimated_acc_params)
    print("residual of those params: ", residual)
    print("errors:\n", estimated_acc_params - acc_params)
    print("error: ", np.sum((estimated_acc_params - acc_params)**2))

    if args.plot:
        data = np.array(data)
        plt.title("fake accelerometer data")
        plt.plot(data[:, 0], label='accel x')
        plt.plot(data[:, 1], label='accel y')
        plt.plot(data[:, 2], label='accel z')
        plt.ylabel("acceleration (gs)")
        plt.xlabel("sample over time")

        xs, ys = double_integrate_acc(data[:, 0:3], 0.01, np.eye(3), np.eye(3), np.array([[0, 0, 0]]))
        plt.figure()
        plt.scatter(xs, ys, s=1, label='raw')
        est_T = np.array([[1, -estimated_acc_params[0], estimated_acc_params[1]], [0, 1, -estimated_acc_params[2]], [0, 0, 1]])
        est_K = np.array([[estimated_acc_params[3], 0, 0], [0, estimated_acc_params[4], 0], [0, 0, estimated_acc_params[5]]])
        est_b = np.array([[estimated_acc_params[6], estimated_acc_params[7], estimated_acc_params[8]]])
        est_xs, est_ys = double_integrate_acc(data[:, 0:3], 0.01, est_K, est_T, est_b)
        plt.scatter(est_xs, est_ys, s=50, label='estimated')
        true_T = np.array([[1, -acc_params[0], acc_params[1]], [0, 1, -acc_params[2]], [0, 0, 1]])
        true_K = np.array([[acc_params[3], 0, 0], [0, acc_params[4], 0], [0, 0, acc_params[5]]])
        true_b = np.array([[acc_params[6], acc_params[7], acc_params[8]]])
        true_xs, true_ys = double_integrate_acc(data[:, 0:3], 0.01, true_K, true_T, true_b)
        plt.scatter(true_xs, true_ys, s=1, label='true')
        plt.legend()
        plt.axis("square")

        plt.show()

    return 0


if __name__ == '__main__':
    sys.exit(main())
