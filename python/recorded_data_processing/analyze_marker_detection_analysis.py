#!/usr/bin/env python3
import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('infile', help='output of marker_detection_analysis program')
    args = parser.parse_args()

    reader = csv.reader(open(args.infile, 'r'))
    data = []
    for line in reader:
        data.append([int(d) for d in line])
    data = np.array(data)

    plt.figure()
    plt.hist(data[:, 1], bins=100)
    plt.xticks(range(0, 100, 2))
    plt.ylabel("Number of detections")
    plt.xlabel("Tag ID #")
    plt.title("Tags Detected")

    distinct_times = [0]
    last_distinct_idx = 0
    for idx in range(1, data.shape[0]):
        if data[idx, 0] != data[last_distinct_idx, 0]:
            dt_s = (data[idx, 0] - data[last_distinct_idx, 0]) / 1e7
            distinct_times.append(dt_s)
            last_distinct_idx = idx
    distinct_times = np.array(distinct_times)

    plt.figure()
    plt.plot(distinct_times)
    plt.ylabel("time since last detected tag")
    plt.xlabel("instances of detected tags")
    plt.title("time between detected tags")

    print(np.mean(distinct_times))
    print(np.median(distinct_times))
    print(np.max(distinct_times))

    plt.show()


if __name__ == '__main__':
    main()