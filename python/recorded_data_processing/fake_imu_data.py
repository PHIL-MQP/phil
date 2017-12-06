import argparse
import numpy as np
import sys
from imu_calibration import optimize, compute_residual
import matplotlib.pyplot as plt
import csv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("outfile", help="output file (csv)")
    parser.add_argument('--seconds', type=int, default=8, help='length of interval at 100 samples per second')
    parser.add_argument('--intervals', type=int, default=18, help='number of intervals')
    parser.add_argument('--noise', type=float, default=1e-5, help='variance of the gaussian noise added')
    parser.add_argument('--seed', type=int, default=1, help='seed numpy random generator')
    parser.add_argument('--plot', action="store_true", help='show a plot of the generated data')
    args = parser.parse_args()

    writer = csv.writer(open(args.outfile, 'w'))

    np.random.seed(args.seed)  # for reproducibility
    acc_params = [0.1, .01, -.01, 1, 0.95, 1.08, -0.01, 0.03, 0.05]
    gyro_params = [0.1, .01, -.01, 1, 0.95, 1.08, -0.01, 0.03, 0.05]

    # generate fake sample of sitting flat on table
    true_a = np.array([[0, 0, 1], [0, 1, 0], [1, 0, 0]]*6, dtype=np.float64)
    true_g = np.array([[0, 0, 0]]*args.intervals, dtype=np.float64)

    noise = np.random.normal(0, args.noise, 3)

    num_samples = args.seconds * 100
    data = []
    fake_mean_accs = np.ndarray((args.intervals, 3))
    for interval in range(args.intervals):
        for i in range(num_samples):
            # skew this data such that our above params would correctly transform it
            Ta = np.array([[1, -acc_params[0], acc_params[1]], [0, 1, -acc_params[2]], [0, 0, 1]])
            Ka = np.array([[acc_params[3], 0, 0], [0, acc_params[4], 0], [0, 0, acc_params[5]]])
            ba = np.array([[acc_params[6]], [acc_params[7]], [acc_params[8]]])
            skewed_a = np.linalg.inv(Ka) @ np.linalg.inv(Ta) @ true_a[interval] - ba.T + noise
            Tg = np.array([[1, -gyro_params[0], gyro_params[1]], [0, 1, -gyro_params[2]], [0, 0, 1]])
            Kg = np.array([[gyro_params[3], 0, 0], [0, gyro_params[4], 0], [0, 0, gyro_params[5]]])
            bg = np.array([[gyro_params[6]], [gyro_params[7]], [gyro_params[8]]])
            skewed_g = np.linalg.inv(Kg) @ np.linalg.inv(Tg) @ true_g[interval] - bg.T + noise

            row = [skewed_a[0][0], skewed_a[0][1], skewed_a[0][2], skewed_g[0][0], skewed_g[0][1], skewed_g[0][2], 0]
            data.append(row)
            writer.writerow(row)
        fake_mean_accs[interval] = np.mean(data[interval*num_samples:(interval+1)*num_samples], axis=0)[0:3]

    estimated_acc_params, residual = optimize(args.intervals, fake_mean_accs)
    print("true parameters:\n", acc_params)
    print("residual of those params: ", compute_residual(args.intervals, fake_mean_accs, acc_params))
    print("paramaters found:\n", estimated_acc_params)
    print("residual of those params: ", residual)
    print("errors:\n", estimated_acc_params - acc_params)
    print("error: ", np.sum((estimated_acc_params - acc_params)**2))

    if args.plot:
        data = np.array(data)
        fig, ax = plt.subplots(1, 2)
        ax[0].set_title("fake accelerometer data")
        ax[0].plot(data[:, 0], label='accel x')
        ax[0].plot(data[:, 1], label='accel y')
        ax[0].plot(data[:, 2], label='accel z')
        ax[0].set_ylabel("acceleration (gs)")
        ax[0].set_xlabel("sample over time")

        ax[1].set_title("fake gyro data")
        ax[1].plot(data[:, 3], label='gyro x')
        ax[1].plot(data[:, 4], label='gyro y')
        ax[1].plot(data[:, 5], label='gyro z')
        ax[1].set_ylabel("rate (degrees/s)")
        ax[1].set_xlabel("sample over time")
        plt.show()

    return 0


if __name__ == '__main__':
    sys.exit(main())
