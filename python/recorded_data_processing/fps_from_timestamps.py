#!/usr/bin/env python3

import numpy as np
import argparse
import matplotlib.pyplot as plt


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Compute FPS statistics from file of timestamps. Assumes microseconds")
    parser.add_argument("timestamps", help="The timestamps file output by mocap_record")
    parser.add_argument("--no-plot", '-p', action="store_true", help="plot FPS over time")
    args = parser.parse_args()

    timestamps = np.genfromtxt(args.timestamps)
    dt = 1e6 / (timestamps[1:] - timestamps[0:-1])
    mean_fps = np.mean(dt)
    median_fps = np.median(dt)
    max_fps = np.max(dt)
    min_fps = np.min(dt)
    print("Parsing for:", args.timestamps);
    print("mean,median,min,max")
    print("{:0.2f}, {:0.2f}, {:0.2f}, {:0.2f}".format(mean_fps, median_fps, min_fps, max_fps))

    if not args.no_plot:
        plt.style.use("python/phil.mplstyle")
        plt.figure()
        plt.plot(dt)
        plt.ylabel("time since last frame (fps)")
        plt.xlabel("frames over time")
        plt.show()
