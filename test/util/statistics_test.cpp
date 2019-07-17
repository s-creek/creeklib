#include <scl/util/Statistics.hpp>
#include <scl/util/EigenUtil.hpp>

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

int main ()
{
    // file open
    std::ifstream file("../data/sample_data.log");
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


    // convert data
    const std::size_t num(dataset.size()), dim(dataset.front().size());
    Eigen::MatrixXd mat(scl::toEigenMatrix(dim, dataset));
    

    // calc covariance
    Eigen::MatrixXd covariance;
    scl::calcCovariance(mat, covariance);
    PRINT_MAT(covariance);
    

    // check first data
    Eigen::VectorXd x(mat.row(0));
    PRINT_MAT(x);


    // calc mean
    Eigen::VectorXd mean(mat.colwise().mean());
    PRINT_MAT(mean);
        

    for (std::size_t i = 0; i < num; ++i)
    {
        double pdf = scl::normal::probabilityDensityFunction(mat.row(i), mean, covariance);
        std::cout << " " << log(pdf);
    }
    std::cout << std::endl;


    double log_likelihood = scl::normal::calcLogLikelihood(mat);
    std::cout << "------------\nresult\n  " << log_likelihood << std::endl; 


    double likelihood = scl::normal::calcLikelihood(mat);
    std::cout << "------------\nresult\n  " << likelihood << std::endl;
    std::cout << "  " << std::log(likelihood) << std::endl;

    
    return 0;
}
