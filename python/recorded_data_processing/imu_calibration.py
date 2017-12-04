import csv
import numpy as np
import ad as autodiff
from scipy.optimize import root
import matplotlib.pyplot as plt
import os
import sys
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('data_file', help='csv file of recorded IMU data')

    args = parser.parse_args()

    reader = csv.reader(open(args.data_file, 'r'))
    next(reader)  # skip header
    data = []
    for row in reader:
        data.append([float(x) for x in row])
    data = np.array(data)

    # Example of LM optimization
    theta_acc = autodiff.adnumber(np.zeros(9), 'theta_acc')
    T = np.array([[1, -theta_acc[0], theta_acc[1]],
                  [0, 1, -theta_acc[2]],
                  [0, 0, 1]])
    K = np.array([[theta_acc[3], 0, 0],
                  [0, theta_acc[4], 0, 0],
                  [0, 0, theta_acc[5]]])
    b = np.array([[theta_acc[6], theta_acc[7], theta_acc[8]]])

    # let c = a @ b
    a = autodiff.adnumber(np.array([0, 0, 0]), 'a')
    b = autodiff.adnumber(np.array([[2, 4, 1], [1, 5, 2], [4, 6, 3]]), 'b')
    c = a * b + a  # confusingly, this does matrix multiplication
    print(c.gradient([a]))


    def accelerometer_error_and_jacobian(a):
        params = autodiff.adnumber(a, 'params')
        f = 9.8 - T @ K @ (a - b)
        df = params.gradient(theta_acc)
        return f, df

    # sol = root(accelerometer_error_and_jacobian, theta_acc, jac=True, method='lm')
    # print(sol.x)


if __name__ == '__main__':
    sys.exit(main())
