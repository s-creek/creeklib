// -*- coding: utf-8; tab-width: 4; mode: c++ -*-

/**
 * @file Statistics.hpp
 */

#ifndef SCL_STATISTICS_HPP
#define SCL_STATISTICS_HPP

#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/LU>

namespace scl
{
    /**
     * @brief calc covariance-matrix
     * @param[in] data_set dataset (num data x dim)
     * @param[out] covariance covariance-matrix
     * @return size check
     */
    bool calcCovariance(const Eigen::MatrixXd &data_set, Eigen::MatrixXd &covariance)
    {
        if ( data_set.rows() < 2 )
        {
            return false;
        }

        Eigen::MatrixXd centered = data_set.rowwise() - data_set.colwise().mean();
        covariance = (centered.adjoint() * centered) / double(data_set.rows() - 1);

        return true;
    }


    /**
     * @brief probability density function
     */
    double calcPdf(const Eigen::VectorXd &x, const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance)
    {
        // set parameter
        constexpr double two_pi(2.0 * M_PI);
        double p(x.size());
        double det(covariance.determinant());
        Eigen::MatrixXd cov(covariance);
        Eigen::VectorXd err(x - mean);

        // calc value
        double alpha = std::pow(two_pi, -0.5*p) * std::pow(det, -0.5);
        double beta = -0.5 * err.transpose() * cov.inverse() * err;

        return ( alpha * std::exp(beta) );
    }


    /**
     * @brief 対数尤度
     * @note log(likelihood) = sum( log(pdf) )
     */
    double calcLogLikelihood(const Eigen::MatrixXd &data_set)
    {
        // calc parameter
        Eigen::VectorXd mean(data_set.colwise().mean());
        Eigen::MatrixXd covariance;
        calcCovariance(data_set, covariance);

        // calc likelihood
        const std::size_t num(data_set.rows());
        double log_likelihood(0.0);
        for (std::size_t i = 0; i < num; ++i)
        {
            double pdf = calcPdf(data_set.row(i), mean, covariance);
            log_likelihood += std::log(pdf);
        }
        return log_likelihood;
    }


    /**
     * @brief 尤度
     * @details 尤度は確率(密度)を標本個数分だけ掛けてできるもの。
     * 0〜1の間の数で何回もかけ算することになり、ほぼ０になってしまう。
     * かけ算が足し算にできる計算のしやすさもあり対数尤度とすることが多い。
     * @see scl::calcLogLikelihood
     * @note likelihood = prod(pdf)
     */
    double calcLikelihood(const Eigen::MatrixXd &data_set)
    {
        // calc parameter
        Eigen::VectorXd mean(data_set.colwise().mean());
        Eigen::MatrixXd covariance;
        calcCovariance(data_set, covariance);

        // calc likelihood
        const std::size_t num(data_set.rows());
        double likelihood(1.0);
        for (std::size_t i = 0; i < num; ++i)
        {
            double pdf = calcPdf(data_set.row(i), mean, covariance);
            likelihood *= pdf;
        }
        return likelihood;
    }


    /**
     * @brief convert matrix data STL -> Eigen
     */
    template<class DataType>
    void convertStlToEigen(const std::size_t dim, const std::vector<DataType> & stl_mat, Eigen::MatrixXd &eigen_mat)
    {
        const std::size_t num( stl_mat.size() );
        if (num < 2)
        {
            return;
        }

        // copy
        eigen_mat.resize(num, dim);
        for (std::size_t i = 0; i < num; ++i)
        {
            for (std::size_t j = 0; j < dim; ++j)
            {
                eigen_mat(i,j) = stl_mat[i][j];
            }
        }
    }


    /**
     * @brief convert matrix data STL -> Eigen
     */
    template<class DataType>
    void convertStlToEigen(const std::size_t dim, const std::vector<DataType> & stl_mat, const std::vector<std::size_t> &indices, Eigen::MatrixXd &eigen_mat)
    {
        const std::size_t num( indices.size() );
        if (num < 2)
        {
            return;
        }

        // copy
        eigen_mat.resize(num, dim);
        for (std::size_t i = 0; i < num; ++i)
        {
            std::size_t index(indices[i]);
            for (std::size_t j = 0; j < dim; ++j)
            {
                eigen_mat(i,j) = stl_mat[index][j];
            }
        }
    }
}

#endif
