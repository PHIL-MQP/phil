import argparse
from time import sleep

from networktables import NetworkTables
import logging
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser("print robot pose")
    parser.add_argument('ntserver', help='the IP of the network tables server')
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG)

    NetworkTables.initialize(server=args.ntserver)

    input("press enter to exit")

    # while not NetworkTables.isConnected():
    #     sleep(1)
    #     print("waiting...")
    #
    # table = NetworkTables.getTable('Root')
    # phil_table = table.getSubTable("phil_table")
    # phil_table2 = NetworkTables.getTable("phil_table")
    #
    # print(phil_table.getNumber("x", -999))
    # print(phil_table2.getNumber("x", -999))


if __name__ == '__main__':
    main()
