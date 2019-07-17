#!/usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np
from scipy import stats


### main
#
if __name__ == '__main__':

    dataset = np.loadtxt("../data/sample_data.log")

    cov = np.cov(dataset.T)
    print "cov :"
    print cov

    x = dataset[0]
    print "x : ", x

    mean = np.mean(dataset, axis=0)
    print "mean : ", mean

    num = dataset.shape[0]
    like = [ stats.multivariate_normal.logpdf(dataset[i], mean, cov) for i in range(num) ]
    print like

    ret = sum(like)
    print "-----------------"
    print "result"
    print "  ", ret
