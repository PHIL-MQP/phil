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
    theta_acc = np.zeros(9, 1)

    def accelerometer_error_and_jacobian(a):
        return a, 1

    sol = root(accelerometer_error_and_jacobian, theta_acc, jac=True, method='lm')
    print(sol.x)


if __name__ == '__main__':
    sys.exit(main())
