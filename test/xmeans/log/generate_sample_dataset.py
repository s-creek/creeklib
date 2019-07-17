#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from scipy import stats
from sklearn.datasets import make_blobs

if __name__ == "__main__":

    #
    # dataset 1
    #
    x = np.array([np.random.normal(loc, 0.1, 20) for loc in np.repeat([1,2], 2)]).flatten()
    y = np.array([np.random.normal(loc, 0.1, 20) for loc in np.tile([1,2], 2)]).flatten()
    dataset = np.c_[x,y]
    np.savetxt("./sample_dataset_01.log", dataset)


    #
    # dataset 2
    #
    X, y = make_blobs(n_samples=500,
                  n_features=2,
                  centers=5,
                  cluster_std=0.8,
                  center_box=(-10.0, 10.0),
                  shuffle=True,
                  random_state=1)  # For reproducibility

    x = X[:,0]
    y = X[:,1]
    dataset = np.c_[x,y]
    np.savetxt("./sample_dataset_02.log", dataset)


    #
    # dataset 3
    #
    X, y = make_blobs(n_samples=500,
                  n_features=2,
                  centers=8,
                  cluster_std=1.5,
                  center_box=(-10.0, 10.0),
                  shuffle=True,
                  random_state=1)  # For reproducibility

    x = X[:,0]
    y = X[:,1]
    dataset = np.c_[x,y]
    np.savetxt("./sample_dataset_03.log", dataset)


    
