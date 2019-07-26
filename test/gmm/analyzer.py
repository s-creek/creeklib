#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import sklearn
from sklearn.mixture import GMM
import scipy
from scipy import linalg


def log_multivariate_normal_density_full(X, means, covars, min_covar=1.e-7):
    """Log probability for full covariance matrices.
    """
    n_samples, n_dim = X.shape
    nmix = len(means)
    log_prob = np.empty((n_samples, nmix))
    for c, (mu, cv) in enumerate(zip(means, covars)):
        try:
            cv_chol = linalg.cholesky(cv, lower=True)
        except linalg.LinAlgError:
            # The model is most probably stuck in a component with too
            # few observations, we need to reinitialize this components
            cv_chol = linalg.cholesky(cv + min_covar * np.eye(n_dim),
                                      lower=True)
        print ""
        print cv_chol
        cv_log_det = 2 * np.sum(np.log(np.diagonal(cv_chol)))
        cv_sol = linalg.solve_triangular(cv_chol, (X - mu).T, lower=True).T
        print ""
        print cv_sol
        print ""
        print cv_log_det
        print ""
        print (X - mu)
        print X
        print mu
        print "------------------------"
        log_prob[:, c] = - .5 * (np.sum(cv_sol ** 2, axis=1) +
                                 n_dim * np.log(2 * np.pi) + cv_log_det)

    return log_prob

    
if __name__ == "__main__":
    X = np.loadtxt("../xmeans/log/sample_dataset_01.log")
    means = np.array( [ [2.03310016, 1.51106166], [1.00107546, 1.49513771] ] )
    covars = np.array([[[2.81814890e-01, 2.62308875e-04],   \
                        [2.62308875e-04, 2.65856936e-01]],  \
                       [[2.81814890e-01, 2.62308875e-04],   \
                        [2.62308875e-04, 2.65856936e-01]]])

    log_prob = log_multivariate_normal_density_full(X[0:1], means, covars)
    log_pdf = [ scipy.stats.multivariate_normal.logpdf(X[0:1], m, c) for m,c in zip(means, covars) ]

    # print X[0:1]
    # print means
    # print covars
    print ""
    print log_prob
    print log_pdf
