#!/usr/bin/env python3
import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy import stats


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('detected_markers', help='output of marker_detection_analysis program (such as detected_markers.csv)')
    parser.add_argument('ids', help='file of ids that you want to allow (filters out others)')
    parser.add_argument('--no-plot', '-p', action="store_true", help='show the plots')
    args = parser.parse_args()

    ids = np.genfromtxt(args.ids, delimiter=',', dtype=np.int32)
    reader = csv.reader(open(args.detected_markers, 'r'))
    data = []
    for line in reader:
        if int(line[1]) in ids:
            data.append([int(d) for d in line[:2]])
    data = np.array(data)

    times_between_detections = [0]
    last_distinct_idx = 0
    for idx in range(1, data.shape[0]):
        if data[idx, 0] != data[last_distinct_idx, 0]:
            dt_s = (data[idx, 0] - data[last_distinct_idx, 0]) / 1e6
            times_between_detections.append(dt_s)
            last_distinct_idx = idx
    times_between_detections = np.array(times_between_detections)

    mean = np.mean(times_between_detections)
    median = np.median(times_between_detections)
    maximum = np.max(times_between_detections)
    mode = stats.mode(times_between_detections)[0][0]
    percentile = np.percentile(times_between_detections, 95)
    print("worst_case,percentile,mean,median,mode")
    print("{:0.3f},{:0.3f},{:0.3f},{:0.3f},{:0.3f}".format(maximum, percentile, mean, median, mode))

    if not args.no_plot:
        # pretty formatting
        mpl.rcParams['axes.formatter.useoffset'] = False
        mpl.rcParams['axes.labelsize'] = 14
        mpl.rcParams['xtick.labelsize'] = 14
        mpl.rcParams['ytick.labelsize'] = 14

        plt.figure()
        max_id = np.max(ids)
        plt.hist(data[:, 1], bins=max_id)
        plt.xticks(range(0, max_id, 2))
        plt.ylabel("Number of detections")
        plt.xlabel("Tag ID #")
        plt.title("Tags Detected", fontsize=18)

        plt.figure()
        plt.plot(times_between_detections)
        plt.ylabel("time since last detected tag")
        plt.xlabel("instances of detected tags")
        plt.title("Time Between Detected Tags", fontsize=18)

        plt.show()


if __name__ == '__main__':
    main()
