#!/usr/bin/python3
import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
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


def base_rotation(a):
    """ https://math.stackexchange.com/a/476311/516340 """
    b = np.array([0, 0, 1])
    v = np.cross(a, b)
    c = np.dot(a, b)
    v_x = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + v_x + (v_x @ v_x) * (1 / (1 + c))
    return R


def build_map_from_dots(top_left_dot_poses, dot_offsets):
    map = np.zeros((top_left_dot_poses.shape[0], 6))
    dots = np.zeros((top_left_dot_poses.shape[0] * 3, 3))

    j = 0
    for i, (top_left_dot_pose, dot_offset) in enumerate(zip(top_left_dot_poses, dot_offsets)):
        top_left_dot = top_left_dot_pose[1:4]
        centroid = top_left_dot - dot_offset[1:4]
        right_dot_pose = centroid + dot_offset[4:7]
        bottom_left_dot_pose = centroid + dot_offset[7:]

        dots[j] = top_left_dot
        dots[j + 1] = right_dot_pose
        dots[j + 2] = bottom_left_dot_pose

        # create the y axis in the tag frame as defined by aruco
        tag_y = top_left_dot - bottom_left_dot_pose
        tag_y = tag_y / np.linalg.norm(tag_y)

        # compute the normal vector of the plane those three points lie in
        top_left_to_right = right_dot_pose - top_left_dot
        tag_z = np.cross(tag_y, top_left_to_right)
        tag_z = tag_z / np.linalg.norm(tag_z)

        # now find the axis orthogonal to both the one going from top-left to bottom-left and the normal
        tag_x = np.cross(tag_z, tag_y)
        tag_x = tag_x / np.linalg.norm(tag_x)

        tag_size_mm = 89.2
        tag_center = top_left_dot + (26.87 + tag_size_mm/2.0)*tag_x + (-63.17 - tag_size_mm/2.0)*tag_y
        map[i, 0:3] = tag_center
        # map[i, 3:] = base_rotation(tag_z)

        j += 3

    return map, dots


def main():
    parser = argparse.ArgumentParser("Compute error of aruco estimate pose")
    parser.add_argument("top_left_dot_poses", help="the hand-written csv file listing global positions top-left dots")
    parser.add_argument("dot_offsets", help="the hand-written csv file listing offsets of all points for each tag")

    args = parser.parse_args()

    dot_offsets = np.genfromtxt(args.dot_offsets, delimiter=',', skip_header=True)
    top_left_dot_poses = np.genfromtxt(args.top_left_dot_poses, delimiter=',', skip_header=True)

    if dot_offsets.shape[0] != top_left_dot_poses.shape[0]:
        print("number of tags don't match. Aborting.")
        return

    map, dots = build_map_from_dots(top_left_dot_poses, dot_offsets)

    recorded_data_processing_dir = os.path.dirname(os.path.realpath(__file__))
    style = recorded_data_processing_dir + "/../phil.mplstyle"
    plt.style.use(style)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_aspect('equal')
    ax.scatter(dots[:, 0], dots[:, 1], dots[:, 2])
    ax.scatter(map[:, 0], map[:, 1], map[:, 2])
    set_axes_equal(ax)
    plt.show()


if __name__ == '__main__':
    sys.exit(main())
