#!/usr/bin/python3
import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import yaml
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D


# https://stackoverflow.com/a/31364297/3353601
def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def get_center_tag_poses_from_map(markermap_yml, ids):
    markers = markermap_yml['aruco_bc_markers']
    measured_map = np.zeros((len(ids), 13))

    j = 0
    for marker in markers:
        tag_id = marker['id']
        if tag_id in ids:
            corners = np.array(marker['corners'])
            # midpoint in 3d of opposite corners (0, 2) or (1, 3) give the center of the tag so we average them
            tag_center = np.sum(corners, axis=0) / 4
            tag_x = corners[1] - corners[0]
            tag_y = corners[0] - corners[3]
            tag_z = np.cross(tag_x, tag_y)
            tag_x = tag_x / np.linalg.norm(tag_x)
            tag_y = tag_y / np.linalg.norm(tag_y)
            tag_z = tag_z / np.linalg.norm(tag_z)
            measured_map[j, 0:3] = tag_center
            measured_map[j, 3:6] = tag_x
            measured_map[j, 6:9] = tag_y
            measured_map[j, 9:12] = tag_z
            measured_map[j, 12] = tag_id
            j += 1

    return measured_map


def get_center_tag_poses_from_dots(top_left_dot_poses, dot_offsets):
    true_map = np.zeros((top_left_dot_poses.shape[0], 13))
    dots = np.zeros((top_left_dot_poses.shape[0] * 3, 3))
    ids = top_left_dot_poses[:, 0]

    j = 0
    for i, (top_left_dot_pose, dot_offset, tag_id) in enumerate(zip(top_left_dot_poses, dot_offsets, ids)):
        top_left_dot = top_left_dot_pose[1:4]
        centroid = top_left_dot - dot_offset[1:4]
        right_dot_pose = centroid + dot_offset[4:7]
        bottom_left_dot_pose = centroid + dot_offset[7:]

        dots[j] = top_left_dot / 1000
        dots[j + 1] = right_dot_pose / 1000
        dots[j + 2] = bottom_left_dot_pose / 1000

        # create the y axis in the tag frame as defined by aruco
        tag_y = top_left_dot - bottom_left_dot_pose
        tag_y = tag_y / np.linalg.norm(tag_y)

        # compute the normal vector of the plane those three points lie in
        top_left_to_right = right_dot_pose - top_left_dot
        tag_z = np.cross(top_left_to_right, tag_y)
        tag_z = tag_z / np.linalg.norm(tag_z)

        # now find the axis orthogonal to both the one going from top-left to bottom-left and the normal
        tag_x = np.cross(tag_y, tag_z)
        tag_x = tag_x / np.linalg.norm(tag_x)

        tag_size_mm = 89.2
        tag_center = top_left_dot + (26.87 + tag_size_mm / 2.0) * tag_x + (-63.17 - tag_size_mm / 2.0) * tag_y
        true_map[i, 0:3] = tag_center / 1000  # convert to meters
        true_map[i, 3:6] = tag_x
        true_map[i, 6:9] = tag_y
        true_map[i, 9:12] = tag_z
        true_map[i, 12] = tag_id

        j += 3

    return true_map, dots, ids


def transform_map(measured_map, true_map, tags_to_make_match):
    """ https://gamedev.stackexchange.com/a/26085 """
    true_origin = true_map[tags_to_make_match]
    measured_origin = measured_map[tags_to_make_match]
    new_map = np.copy(measured_map)  # important to copy the tag ids

    A = measured_origin[3:12].reshape((3, 3)).T
    B = true_origin[3:12].reshape((3, 3)).T

    R1 = B @ np.linalg.inv(A)
    new_map[:, 0:3] = (measured_map[:, 0:3] - measured_origin[0:3]).dot(R1.T) + true_origin[0:3]
    new_map[:, 3:6] = measured_map[:, 3:6].dot(R1.T)
    new_map[:, 6:9] = measured_map[:, 6:9].dot(R1.T)
    new_map[:, 9:12] = measured_map[:, 9:12].dot(R1.T)

    return new_map


def angle_error(u, v):
    """ https://stackoverflow.com/a/13849249/3353601, output is in degrees """
    u = u / np.linalg.norm(u)
    v = v / np.linalg.norm(v)
    return np.rad2deg(np.arccos(np.clip(np.dot(v, u), -1.0, 1.0)))


def main():
    parser = argparse.ArgumentParser("Compute error of aruco estimate pose")
    parser.add_argument("top_left_dot_poses", help="the hand-written csv file listing global positions top-left dots")
    parser.add_argument("dot_offsets", help="the hand-written csv file listing offsets of all points for each tag")
    parser.add_argument("markermap", help="a map.yml file output form makermapper_from_video")
    parser.add_argument("--no-plot", help="don't plot", action="store_true")
    parser.add_argument("--verbose", help="print more information", action="store_true")
    parser.add_argument("--tags-to-make-match", help="the ID of the tags should be made perfectly aligned", type=int,
                        default=0)

    args = parser.parse_args()

    compute_error(args)


def compute_error(args):
    dot_offsets = np.genfromtxt(args.dot_offsets, delimiter=',', skip_header=True)
    top_left_dot_poses = np.genfromtxt(args.top_left_dot_poses, delimiter=',', skip_header=True)
    markermap_file = open(args.markermap, 'r')
    markermap_file.readline()  # skip the first line because it's invalid YAML
    markermap_yml = yaml.load(markermap_file)

    if dot_offsets.shape[0] != top_left_dot_poses.shape[0]:
        print("number of tags don't match. Aborting.")
        return

    true_map, dots, ids = get_center_tag_poses_from_dots(top_left_dot_poses, dot_offsets)
    measured_map = get_center_tag_poses_from_map(markermap_yml, ids)

    # the measured map is relative to tag 0, so we need to transform everything to align with tag 0 according to mocap
    all_translational_errors = []
    all_per_tag_translational_errors = []
    all_per_tag_rotational_errors = []

    print("translation error to tag 0, per tag translational error, rotational error per tag")

    for tags_to_make_match in ids:
        idx_to_make_match = np.where(ids == tags_to_make_match)[0][0]
        measured_map = transform_map(measured_map, true_map, idx_to_make_match)

        #
        # Error analysis
        #
        if args.verbose:
            print("Transforming to make tags {:d} line up".format(tags_to_make_match))

        # find the distances between each tag and tag 0, and compare these distances between mocap & markermapper
        N = true_map.shape[0]
        errors = np.zeros((N, N))
        if args.verbose:
            print("\nTranslation errors on to tag 0")
        for tag_to_measure_to in range(N):
            for i, (true_marker, measured_marker) in enumerate(zip(true_map, measured_map)):
                disp_true = np.linalg.norm(true_marker[0:3] - true_map[tag_to_measure_to, 0:3])
                disp_measured = np.linalg.norm(measured_marker[0:3] - measured_map[tag_to_measure_to, 0:3])
                errors[tag_to_measure_to, i] = abs(disp_measured - disp_true)
            if args.verbose:
                print(int(ids[tag_to_measure_to]), "{:0.3f}".format(errors[tag_to_measure_to, 0]))
        if args.verbose:
            print()

        # find the "error" distance for tag N. This is the distance between mocap and markermapper
        if args.verbose:
            print("\nTranslation errors on each tag")
        per_tag_translational_errors = np.ndarray(N)
        for i, (true_marker, measured_marker) in enumerate(zip(true_map, measured_map)):
            distance_error = np.linalg.norm(true_marker[0:3] - measured_marker[0:3])
            if args.verbose:
                print(int(ids[i]), "{:0.3f}".format(distance_error))
            per_tag_translational_errors[i] = distance_error
        if args.verbose:
            print()

        # find the rotational error of each axis
        if args.verbose:
            print("\nRotational errors on each tag")
            print("X   Y")
        per_tag_rotational_errors = np.ndarray((N, 2))
        for i, (true_marker, measured_marker) in enumerate(zip(true_map, measured_map)):
            error_x = angle_error(true_marker[3:6], measured_marker[3:6])
            error_z = angle_error(true_marker[9:12], measured_marker[9:12])
            if args.verbose:
                print(int(ids[i]), "{:0.3f}, {:0.3f}".format(error_x, error_z))
            per_tag_rotational_errors[i, 0] = error_x
            per_tag_rotational_errors[i, 1] = error_z

        print("{:.3f}, {:.3f}, {:.3f}".format(errors.mean(), np.mean(per_tag_translational_errors),
                                              np.mean(per_tag_rotational_errors)))
        all_translational_errors.append(errors.mean())
        all_per_tag_translational_errors.append(np.mean(per_tag_translational_errors))
        all_per_tag_rotational_errors.append(np.mean(per_tag_rotational_errors))

        #
        # Plotting
        #

        if args.no_plot:
            continue

        # only plot for one or it will be confusing
        if tags_to_make_match != args.tags_to_make_match:
            continue

        recorded_data_processing_dir = os.path.dirname(os.path.realpath(__file__))
        style = recorded_data_processing_dir + "/../phil.mplstyle"
        plt.style.use(style)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_aspect('equal')
        ax.scatter(true_map[:, 0], true_map[:, 1], true_map[:, 2], label="mocap", c='green')
        ax.scatter(measured_map[:, 0], measured_map[:, 1], measured_map[:, 2], label="markermap", c='red')
        ax.quiver(true_map[:, 0], true_map[:, 1], true_map[:, 2], true_map[:, 3], true_map[:, 4], true_map[:, 5],
                  length=0.3, arrow_length_ratio=0.5, color='red')
        ax.quiver(true_map[:, 0], true_map[:, 1], true_map[:, 2], true_map[:, 6], true_map[:, 7], true_map[:, 8],
                  length=0.3, arrow_length_ratio=0.5, color='green')
        ax.quiver(true_map[:, 0], true_map[:, 1], true_map[:, 2], true_map[:, 9], true_map[:, 10], true_map[:, 11],
                  length=0.3, arrow_length_ratio=0.5, color='blue')
        ax.quiver(measured_map[:, 0], measured_map[:, 1], measured_map[:, 2], measured_map[:, 3], measured_map[:, 4],
                  measured_map[:, 5], length=0.3, arrow_length_ratio=0.5, color='red')
        ax.quiver(measured_map[:, 0], measured_map[:, 1], measured_map[:, 2], measured_map[:, 6], measured_map[:, 7],
                  measured_map[:, 8], length=0.3, arrow_length_ratio=0.5, color='green')
        ax.quiver(measured_map[:, 0], measured_map[:, 1], measured_map[:, 2], measured_map[:, 9], measured_map[:, 10],
                  measured_map[:, 11], length=0.3, arrow_length_ratio=0.5, color='blue')
        set_axes_equal(ax)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        plt.legend()

    print("averages:")
    print("{:.3f}, {:.3f}, {:.3f}".format(np.mean(all_translational_errors), np.mean(all_per_tag_translational_errors),
                                          np.mean(all_per_tag_rotational_errors)))

    plt.show()


if __name__ == '__main__':
    sys.exit(main())
