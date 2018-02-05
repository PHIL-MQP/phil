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
    plt.show()


if __name__ == '__main__':
    main()
