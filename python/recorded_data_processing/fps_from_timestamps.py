#!/usr/bin/env python3

import numpy as np
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("timestamps", help="The timestamps file output by mocap_record")
    args = parser.parse_args()

    timestamps = np.genfromtxt(args.timestamps)
    dt = timestamps[1:] - timestamps[0:-1]
    fps = 1000000 / np.mean(dt)
    print("Average FPS:", fps)
