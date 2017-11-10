#!/usr/bin/env python3

import numpy as np
import argparse
import sys
import os
import csv
import json


def encoder_diff(ticks_1, ticks_2, encoder_bits=16):
    """ Like doing ticks_2 - ticks_1, but aware of edge cases """
    diff = ticks_2 - ticks_1
    if diff > 2 ** (encoder_bits - 1):  # encoder actually went backwards and wrapped around (ex: 1, 65535)
        return -ticks_1 - ((2 ** encoder_bits + 1) - ticks_2)
    elif diff < -2 ** (encoder_bits - 1):  # encoder went forwards and wrapped around (ex: 65535, 1)
        return (2 ** encoder_bits + 1) - ticks_1 + ticks_2
    return diff


def load_files(data_directory):
    metadata_filename = os.path.join(data_directory, "metadata.json")
    encoder_filename = os.path.join(data_directory, "encoder_data.csv")
    imu_filename = os.path.join(data_directory, "ins_data.csv")

    metadata = json.load(open(metadata_filename, 'r'))
    encoder_file = open(encoder_filename, 'r')
    imu_file = open(imu_filename, 'r')
    encoder_reader = csv.reader(encoder_file)
    imu_reader = csv.reader(imu_file)

    return metadata, encoder_reader, imu_reader


def load_data(data_directory):
    """
    Load the bagged sensor data from the specified directory and return two numpy arrays.
    The first is speeds, and has the size Nx3. The second is NavX, and has the size Mx7
    N/M here are the number of readings.
    """
    metadata, encoder_reader, imu_reader = load_files(data_directory)

    roborio_laptop_time_offset = metadata['laptop_time_s'] - metadata['roborio_time_s']

    next(encoder_reader)  # skip header
    previous_row = next(encoder_reader)
    speeds = []
    ticks_per_wheel_rev = metadata["gear_ratio"] * metadata["ticks_per_motor_rev"]
    for row in encoder_reader:
        left_radians = encoder_diff(float(row[0]), float(previous_row[0])) * 2 * np.pi / ticks_per_wheel_rev
        right_radians = encoder_diff(float(row[1]), float(previous_row[1])) * 2 * np.pi / ticks_per_wheel_rev
        left_speed_rad_per_sec = left_radians / metadata['encoder_period_s']
        right_speed_rad_per_sec = right_radians / metadata['encoder_period_s']
        time = float(row[2])

        speeds.append(np.array([left_speed_rad_per_sec, right_speed_rad_per_sec, time]))

        previous_row = row
    speeds = np.array(speeds)

    imus = []
    next(imu_reader)  # skip header
    for row in imu_reader:
        imu = np.array([float(x) for x in row])
        imu[-2] = imu[-2] + roborio_laptop_time_offset
        imu[-1] = imu[-1] + roborio_laptop_time_offset
        imus.append(imu)
    imus = np.array(imus)

    return speeds, imus, metadata


def interpolate_data(imus, speeds, t):
    """
    given numpy arrays of sensor data, produce a synthetic sensor reading at one specific time.
    The reading will be interpolated between the actual readings.
    """
    # find the two sensor readings nearest to t and linearly interpolate
    sensor_values = []

    # find the interpolated speeds
    points_found = False
    for idx in range(1, len(speeds)):
        t0 = speeds[idx - 1][-1]  # assumes time is last
        t1 = speeds[idx][-1]
        s0 = speeds[idx - 1]
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
        t0 = imus[idx - 1][-1]  # assumes time is last
        t1 = imus[idx][-1]
        s0 = imus[idx - 1]
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
    """ interpolate between (t0, s0) and (t1, s1) at time t """
    m = (s1 - s0) / (t1 - t0)
    return s0 + m * (t - t0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_directory", help="directory with data")

    args = parser.parse_args()

    if not os.path.exists(args.data_directory):
        print("data direction [{}] does not exist".format(args.data_directory))
        return

    output_filename = os.path.join(args.data_directory, "interpolated_data.csv")

    speeds, imus, metadata = load_data(args.data_directory)

    # iterate over all the time imu readings were taken and get interpolated sensor values
    writer = csv.writer(open(output_filename, 'w'))
    writer.writerow(["left", "right", "imux", "imuy", "imuz", "gyrox", "gyroy", "gyroz", "time"])
    t0 = max(speeds[0][-1], imus[0][-1])
    t1 = min(speeds[-1][-1], imus[-1][-1])
    for t in np.arange(t0, t1, 0.1):
        sensor_values = interpolate_data(imus, speeds, t)
        sensor_values.append(t)
        writer.writerow(sensor_values)


if __name__ == '__main__':
    sys.exit(main())
