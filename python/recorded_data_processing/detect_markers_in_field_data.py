#!/usr/bin/env python3
import argparse
import os
from subprocess import call


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', help='path to the field_data_* folder')
    parser.add_argument('camera_calibration', help='path to the camera calibration file')
    args = parser.parse_args()

    subfolders = os.listdir(args.root)
    for folder in subfolders:
        full_path = os.path.join(args.root, folder)
        infile = [f for f in os.listdir(full_path) if f[:4] == "out_"][0]
        full_infile = os.path.join(full_path, infile)
        full_outfile = os.path.join(full_path, 'annotated-' + infile)
        args = ['./cpp/.build/tools/detect_markers', full_infile, args.camera_calibration, '-q', full_outfile]
        print('calling:', args)
        call(args)


if __name__ == '__main__':
    main()
