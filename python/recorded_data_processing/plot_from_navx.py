#!/usr/bin/env python3

import argparse
import csv
import os

import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np


def load_motion_capture_csv(robot_name, mocap_reader):
    next(mocap_reader)
    next(mocap_reader)
    object_names = next(mocap_reader)
    next(mocap_reader)
    next(mocap_reader)

    if robot_name not in object_names:
        return None

    robot_columns = np.arange(6, dtype=np.int32) + int(object_names.index(robot_name))
    robot_poses = []
    for mocap_data in mocap_reader:
        if not mocap_data:
            break
        robot_pose = np.zeros(len(mocap_data))
        for i, d in enumerate(mocap_data):
            try:
                robot_pose[i] = float(d)
            except ValueError:
                pass  # default to 0
        robot_poses.append(robot_pose[robot_columns])
    robot_poses = np.array(robot_poses)
    return robot_poses


def get_static_intervals(threshold, data, window_size):
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
        classifications[center] = 1.5 if static else -1

    if previously_static:
        temp_pair[1] = data_array_size - window_size // 2 - 1
        static_indicators.append(temp_pair)

    return static_indicators, classifications


def find_static_mean(data, static_intervals):
    averages = []

    for x in static_intervals:
        averages.append(np.mean(data[x[0]: x[1], :], axis=0))

    averages = np.array(averages)

    return averages


def read_data(filename):
    reader = csv.reader(open(filename, 'r'))
    next(reader)  # skip header
    data = []
    for line in reader:
        data.append([float(d) for d in line])
    data = np.array(data)
    return data


def find_static_intervals(raw_acc_data, threshold_multiplier):
    num_static_samples = 61
    init_variance = np.linalg.norm(np.var(raw_acc_data[1:num_static_samples, :], axis=0))
    static_threshold = init_variance * threshold_multiplier
    static_intervals, classification = get_static_intervals(static_threshold, raw_acc_data, 60)
    return static_intervals, classification


def calibrate(input_data):
    # Ideal accelerometer calibration parameters
    acc_calibration_params = np.array([2.29299485e-03, 9.73357832e-04, 2.18891523e-03,
                                       9.97372417e-01, 9.98078141e-01, 9.95206414e-01,
                                       -8.12765191e-03, -1.24052008e-02, -1.41327621e-02])

    Ta = np.array([[1, -acc_calibration_params[0], acc_calibration_params[1]],
                   [0, 1, -acc_calibration_params[2]],
                   [0, 0, 1]])

    Ka = np.array([[acc_calibration_params[3], 0, 0],
                   [0, acc_calibration_params[4], 0],
                   [0, 0, acc_calibration_params[5]]])

    ba = acc_calibration_params[6:9]

    return (Ta @ Ka @ (input_data[:, 0:3] + ba).T).T


def base_rotation(mean_acc_while_stationary):
    """ https://math.stackexchange.com/a/476311/516340 """
    a = mean_acc_while_stationary
    b = np.array([0, 0, 1])
    v = np.cross(a, b)
    c = np.dot(a, b)
    v_x = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + v_x + (v_x @ v_x) * (1 / (1 + c))
    return R


def clockwise_yaw_rotation_matrix(yaw_angle):
    # https://en.wikipedia.org/wiki/Rotation_matrix
    R = np.array([[np.cos(yaw_angle), np.sin(yaw_angle), 0], [-np.sin(yaw_angle), np.cos(yaw_angle), 0], [0, 0, 1]])
    return R


def simple_euler_integration(data, dts, units_conversion, v_i=0):
    result = np.zeros(data.shape)
    for i in range(1, len(data)):
        a = data[i] / 2 + data[i - 1] / 2
        v_f = a * dts * units_conversion + v_i
        result[i] = v_f
        v_i = v_f
    return result


def integrate_velocity_with_yaw(data, dts, units_conversion, yaws_rad, v_i=0):
    result = np.zeros(data.shape)
    for i in range(1, len(data)):
        a = data[i] / 2 + data[i - 1] / 2

        a = (clockwise_yaw_rotation_matrix(yaws_rad[i]) @ a.T).T
        v_f = a * dts * units_conversion + v_i
        result[i] = v_f
        v_i = v_f
    return result


def apply_zero_velocity_updates_and_integrate(rotated_acc_data, static_intervals, yaws_rad, dts):
    for i, interval in enumerate(static_intervals):
        left_bound = interval[0]
        right_bound = interval[1]
        static_interval_mean = np.mean(rotated_acc_data[left_bound: right_bound + 1, :], axis=0)

        bias = [0, 0, 1] - static_interval_mean
        if i != (len(static_intervals) - 1):
            right_dyn_bound = static_intervals[i + 1][0]
            for j in range(left_bound, right_dyn_bound + 1):
                rotated_acc_data[j] += bias
        else:
            for j in range(left_bound, len(rotated_acc_data)):
                rotated_acc_data[j] += bias

    final_velocity_zeroed = np.zeros(rotated_acc_data.shape)
    for i in range(len(static_intervals)):
        left = static_intervals[i][0]
        if i != (len(static_intervals) - 1):
            right = static_intervals[i + 1][0]
            final_velocity_zeroed[left:right] = integrate_velocity_with_yaw(rotated_acc_data[left:right], dts, 9.8,
                                                                            yaws_rad[left:right])
        else:
            final_velocity_zeroed[left:] = integrate_velocity_with_yaw(rotated_acc_data[left:], dts, 9.8,
                                                                       yaws_rad[left:])

    final_displacement = simple_euler_integration(final_velocity_zeroed, dts, 1)

    return final_velocity_zeroed, final_displacement


def main():
    default_robot_name = 'Global Angle phil_turtlebot:phil_turtlebot'

    parser = argparse.ArgumentParser("Takes a rio log file and uses raw accelerometer and yaw to compute position")
    parser.add_argument('rio_data', help='a csv file of logged imu data.')
    parser.add_argument('mocap_csv', help='csv motion capture file')
    parser.add_argument('--initial-yaw-deg', help='initial yaw value (in Degrees', required=True, type=float)
    parser.add_argument('--static-threshold', help='a multiplied to determin static or not', type=float, default=0.01)
    parser.add_argument('--plot-all', help='supersedes other plotting arguments', action="store_true")
    parser.add_argument('--plot-yaw', action="store_true")
    parser.add_argument('--plot-velocity', action="store_true")
    parser.add_argument('--plot-static-intervals', action="store_true")
    parser.add_argument('--robot-name',
                        help='title of the column with the rio data. default is [%s]'.format(default_robot_name),
                        default=default_robot_name)

    args = parser.parse_args()

    #
    # Load the data
    #

    rio_dts = 0.02
    mocap_dts = 0.01

    mocap_reader = csv.reader(open(args.mocap_csv, 'r'))
    mocap_poses = load_motion_capture_csv(args.robot_name, mocap_reader)
    if mocap_poses is None:
        print("There's no column [{:s}] in [{:s}]".format(args.robot_name, args.mocap_csv))
        return
    mocap_xs = (mocap_poses[:, 3] - mocap_poses[0, 3]) / 1000
    mocap_ys = (mocap_poses[:, 4] - mocap_poses[0, 4]) / 1000

    data = read_data(args.rio_data)
    raw_acc_data = data[:, 0:3]
    yaws_rad = np.deg2rad(data[:, 5]) + np.deg2rad(args.initial_yaw_deg)

    #
    # Process the data
    #

    static_intervals, classification = find_static_intervals(raw_acc_data, args.static_threshold)

    calibrated_acc_data = calibrate(raw_acc_data)[:, :3]

    calibrated_mean = np.mean(calibrated_acc_data[static_intervals[0][0]:static_intervals[0][1], :], axis=0)

    unit_calibrated_mean = calibrated_mean / np.linalg.norm(calibrated_mean)

    R = base_rotation(unit_calibrated_mean)

    rotated_acc_data = (R @ calibrated_acc_data.T).T

    velocity, position = apply_zero_velocity_updates_and_integrate(rotated_acc_data, static_intervals, yaws_rad,
                                                                   rio_dts)

    #
    # Plotting
    #

    time = np.arange(data.shape[0]) * rio_dts
    mocap_time = np.arange(mocap_poses.shape[0]) * mocap_dts
    if args.plot_yaw or args.plot_static_intervals or args.plot_all:
        recorded_data_processing_dir = os.path.dirname(os.path.realpath(__file__))
        style = recorded_data_processing_dir + "/../phil.mplstyle"
        plt.style.use(style)

    if args.plot_yaw or args.plot_all:
        plt.figure()
        plt.plot(time, yaws_rad)
        plt.xlabel("samples over time")
        plt.ylabel("yaw (radians)")

    if args.plot_static_intervals or args.plot_all:
        opacity = 0.8
        plt.figure()
        plt.title("Acc raw data with static detector")
        plt.xlabel("# of samples")
        plt.ylabel("acceleration (g)")
        plt.plot(time, raw_acc_data[:, 0], c='y', alpha=opacity, label='acc x')
        plt.plot(time, raw_acc_data[:, 1], c='g', alpha=opacity, label='acc y')
        plt.plot(time, raw_acc_data[:, 2], c='c', alpha=opacity, label='acc z')
        plt.plot(time, classification, c='r', label='detector')
        plt.legend(bbox_to_anchor=(1, 1))

    if args.plot_velocity or args.plot_all:
        plt.figure()
        plt.title("Velocity")
        plt.xlabel("time (s)")
        plt.ylabel("velocity (m/s)")
        plt.plot(time, velocity[:, 0], c='y', label='x velocity')
        plt.plot(time, velocity[:, 1], c='g', label='y velocity')
        plt.legend()

    plt.figure()
    plt.title("Displacement")
    plt.xlabel("x displacement (m)")
    plt.ylabel("y displacement (m)")
    plt.axis("equal")
    colors = cm.rainbow(np.linspace(0, 1, position.shape[0]))
    plt.scatter(position[:, 0], position[:, 1], s=1, color=colors, label='IMU')
    skip = 20
    plt.quiver(position[::skip, 0], position[::skip, 1], np.cos(yaws_rad[::skip]), np.sin(yaws_rad[::skip]), scale=50, width=0.001)
    plt.plot(mocap_xs, mocap_ys, linewidth=1, label='Mocap')
    plt.legend()

    plt.show()


if __name__ == '__main__':
    main()
