#!/usr/bin/env python3

import numpy as np
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("timestamps", help="The timestamps file output by mocap_record")
    args = parser.parse_args()

    timestamps = np.genfromtxt(args.timestamps)
    dt = timestamps[1:] - timestamps[0:-1]
    mean_fps = 1000000 / np.mean(dt)
    median_fps = 1000000 / np.median(dt)
    min_fps = 1000000 / np.max(dt)
    max_fps = 1000000 / np.min(dt)
    print("Mean FPS:", mean_fps)
    print("Median FPS:", median_fps)
    print("Min FPS:", min_fps)
    print("Max FPS:", max_fps)
