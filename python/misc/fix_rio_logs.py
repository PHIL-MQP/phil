#!/usr/bin/env python

import argparse

import sys

import os


def main():
    parser = argparse.ArgumentParser("Script to automatically fix the rio log files where there's a missing comma")
    parser.add_argument('infile', help='input rio csv file')
    parser.add_argument('--overwrite', help="overwrite the input file", action="store_true")

    bad_column = 2
    temp_filename = '/tmp/temp_rio_log.csv'

    args = parser.parse_args()

    with open(args.infile, 'r') as infile:
        if args.overwrite:
            outfile = open(temp_filename, 'w')
        else:
            outfile = sys.stdout

        header = infile.readline()
        header = header.split(',')
        header.remove('fused_heading')
        outfile.write(','.join(header))
        for line_number, line in enumerate(infile.readlines()):
            split = line.split(',')
            broken_cell = split[bad_column]
            if '-' in broken_cell[1:]:
                missing_comma_idx = broken_cell.rindex('-')
                first_num = broken_cell[:missing_comma_idx]
                second_num = broken_cell[missing_comma_idx:]
            elif broken_cell[-1] == '0':
                first_num = broken_cell[:-1]
                second_num = '0'
            elif broken_cell.count('.') == 2:
                missing_comma_idx = broken_cell.rindex('.') - 1
                first_num = broken_cell[:missing_comma_idx]
                second_num = broken_cell[missing_comma_idx:]
            elif broken_cell.count('.') == 1 and broken_cell[0] == '1':
                first_num = '1'
                second_num = broken_cell[1:]
            else:
                print(line_number, broken_cell, 'failed to split')
                return
            split.pop(bad_column)
            split.insert(bad_column, first_num)
            split.insert(bad_column + 1, second_num)
            outfile.write(",".join(split))

        outfile.close()
        if args.overwrite:
            os.rename(temp_filename, args.infile)


if __name__ == '__main__':
    main()
