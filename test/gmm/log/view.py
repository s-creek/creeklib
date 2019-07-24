#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import random


def show(filename):
    # load data
    file_data = open(filename, 'r')
    lines = file_data.readlines()
   
    # plot
    plt.figure()
    plt.title(filename)
    for line in lines:
        split_line = line.strip().split(' ')
        xy = [ float(p) for p in split_line ]
        plt.scatter( xy[0::2], xy[1::2], c = (random.random(), random.random(), random.random()), marker='o', s=50 )


def main():
    show("./clustering_gmm_cpp.log")
    plt.show()
    
    
### main
#
if __name__ == '__main__':
    main()
