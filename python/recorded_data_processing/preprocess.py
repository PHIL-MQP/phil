import argparse
import sys
import os
import csv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("encoders", help="csv file with encoder data")
    parser.add_argument("imu", help="csv file with imu data")

    args = parser.parse_args()

    encoder_file = open(args.encoders, 'r')
    imu_file = open(args.imu, 'r')
    encoder_reader = csv.reader(encoder_file)
    imu_reader = csv.reader(imu_file)

    roborio_laptop_time_offset = 1509598200.0 - 6198.0
    encoder_period_s = 0.1
    imu_period_s = 0.01

    previous_row = next(encoder_reader)
    for row in encoder_reader:
        left_speed = row[0] - previous_row[0]
        right_speed = row[1] - previous_row[1]
        time = row[2]

        previous_row = row

    previous_row = next(imu_reader)
    for row in imu_reader:
        pass


if __name__ == '__main__':
    sys.exit(main())

