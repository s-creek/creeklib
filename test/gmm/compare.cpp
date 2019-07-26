//#include <scl/util/Statistics.hpp>
#include <scl/util/EigenUtil.hpp>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>

#include <vector>
#include <fstream>
#include <sstream>

#include <iostream>   // debug
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

namespace scl {
    namespace normal {
        double probabilityDensityFunction(const Eigen::VectorXd &x, const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance)
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


        double logpdf(const Eigen::VectorXd &x, const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance)
        {
            // set parameter
            constexpr double two_pi(2.0 * M_PI);
            double p(x.size());

            // cholesky decomposition
            Eigen::LLT<Eigen::MatrixXd> cholesky(covariance);
            Eigen::VectorXd err(x - mean);
            Eigen::VectorXd sol;
            Eigen::MatrixXd L = cholesky.matrixL();
            sol = L.triangularView<Eigen::Lower>().solve(err);

            // PRINT_MAT(L);
            // PRINT_MAT(sol.transpose());
            // std::cout << std::log(covariance.determinant()) << std::endl;
            // PRINT_MAT(err.transpose());
            // PRINT_MAT(x.transpose());
            // PRINT_MAT(mean.transpose());
            
            return -0.5 * (sol.squaredNorm() + p * std::log(two_pi) + std::log(covariance.determinant() ) );
        }
    }
}
    
int main (int argc, char **argv)
{
    std::string file_name("../xmeans/log/sample_dataset_01.log");
    if (argc > 1)
    {
        file_name = argv[1];
    }
    
    
    // file open
    std::ifstream file(file_name);
    if ( !file )
    {
        return 0;
    }


    // load data
    std::vector< std::vector<double> > dataset;
    std::string line;
    while ( std::getline(file, line) )
    {
        if ( line.size() > 1 )
        {
            std::stringstream ss(line);
            std::vector<double> data;
            double tmp(0);
            while ( !ss.eof() )
            {
                ss >> tmp;
                data.push_back(tmp);
            }
            dataset.push_back(data);
        }
    }
    file.close();


    // mean
    std::vector< Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > mean( { Eigen::Vector2d({2.03310016, 1.51106166}), Eigen::Vector2d({1.00107546, 1.49513771}) } );
    
    // covariance
    Eigen::MatrixXd cv(2, 2);
    cv << 2.81814890e-01, 2.62308875e-04, 2.62308875e-04, 2.65856936e-01;
    std::vector< Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > covariance(2, cv);

    // weight
    std::vector<double> weight(2, 0.5);


    int dim(2);
    int num_cluster(2);
    Eigen::MatrixXd X( scl::toEigenMatrix(dim, dataset) );
    //for (int i = 0; i < X.rows(); ++i)
    for (int i = 0; i < 1; ++i)
    {
        PRINT_MAT(X.row(i));
        for (int j = 0; j < num_cluster; ++j)
        {
            double pdf = scl::normal::probabilityDensityFunction(X.row(i), mean[j], covariance[j]);
            double logpdf = scl::normal::logpdf(X.row(i), mean[j], covariance[j]);
            //std::cout << "  " << pdf;
            //std::cout << "pdf(" << j << ") = " << std::log(pdf) << std::endl;
            std::cout << std::log(pdf) << ",  " << logpdf << std::endl;
        }
        std::cout << std::endl;
    }
    
    
    return 0;
}
