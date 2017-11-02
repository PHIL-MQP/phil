import numpy as np
import argparse
import sys
import os
import csv
import json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_directory", help="directory with data")

    args = parser.parse_args()

    if not os.path.exists(args.data_directory):
        return

    metadata_filename = os.path.join(args.data_directory, "metadata.json")
    encoder_filename = os.path.join(args.data_directory, "encoder_data.csv")
    imu_filename = os.path.join(args.data_directory, "ins_data.csv")
    output_filename = os.path.join(args.data_directory, "interpolated_data.csv")

    metadata = json.load(open(metadata_filename, 'r'))

    encoder_file = open(encoder_filename, 'r')
    imu_file = open(imu_filename, 'r')
    encoder_reader = csv.reader(encoder_file)
    imu_reader = csv.reader(imu_file)

    roborio_laptop_time_offset = metadata['laptop_time_s'] - metadata['roborio_time_s']

    previous_row = next(encoder_reader)
    speeds = []
    for row in encoder_reader:
        left_speed = float(row[0]) - float(previous_row[0])
        right_speed = float(row[1]) - float(previous_row[1])
        time = float(row[2])

        speeds.append(np.array([left_speed, right_speed, time]))

        previous_row = row

    imus = []
    for row in imu_reader:
        imu = np.array([float(x) for x in row])
        imu[-2] = imu[-2] + roborio_laptop_time_offset
        imu[-1] = imu[-1] + roborio_laptop_time_offset
        imus.append(imu)

    # iterate over all the time imu readings were taken and get interpolated sensor values
    writer = csv.writer(open(output_filename, 'w'))
    for t in np.arange(imus[0][-1], imus[-1][-1], 0.1):
        sensor_values = interpolate_data(imus, speeds, t)
        writer.writerow(sensor_values)


def interpolate_data(imus, speeds, t):
    # find the two sensor readings nearest to t and linearly interpolate
    sensor_values = []

    # find the interpolated speeds
    points_found = False
    for idx in range(1, len(speeds)):
        t0 = speeds[idx-1][-1]  # assumes time is last
        t1 = speeds[idx][-1]
        s0 = speeds[idx-1]
        s1 = speeds[idx]
        if t0 <= t <= t1:
            # linear interpolate
            s = interpolate(s0, s1, t0, t1, t)
            sensor_values[:2] = s[:2]
            points_found = True
            break

    if not points_found:
        raise ValueError("t={} not in interval of encoder values [{}, {}]".format(t, speeds[0][-1], speeds[-1][-1]))

    # find the interpolated imu data
    points_found = False
    for idx in range(1, len(imus)):
        t0 = imus[idx-1][-1]  # assumes time is last
        t1 = imus[idx][-1]
        s0 = imus[idx-1]
        s1 = imus[idx]
        if t0 <= t <= t1:
            # linear interpolate
            s = interpolate(s0, s1, t0, t1, t)
            sensor_values[2:] = s[:-2]
            points_found = True
            break

    if not points_found:
        raise ValueError("t={} not in interval of imu values [{}, {}]".format(t, imus[0][-1], imus[-1][-1]))

    return sensor_values


def interpolate(s0, s1, t0, t1, t):
    m = (s1 - s0) / (t1 - t0)
    return s0 + m * (t - t0)


if __name__ == '__main__':
    sys.exit(main())

