#!/usr/bin/env python3
import argparse
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np


def main():
    parser = argparse.ArgumentParser("Takes output of aruco_localize_from_mm and plots it")
    parser.add_argument("camera_poses", help="space-seperated camera.poses file")
    parser.add_argument("--number_of_points", "-n", type=int, default=100, help="number of lines in the poses file to plot")
    args = parser.parse_args()


    n = args.number_of_points
    poses = np.genfromtxt(args.camera_poses, delimiter=' ')
    colors = cm.rainbow(np.linspace(0, 1, n))
    plt.scatter(poses[:n,1], poses[:n,2], color=colors)
    plt.plot(poses[:n,1],poses[:n,2])
    plt.show()

if __name__ == '__main__':
    main()
