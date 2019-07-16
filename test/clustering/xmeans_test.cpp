#include <scl/clustering/XMeans.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

int main ()
{
    // file open
    std::ifstream file("./log/sample_data.log");
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

    
    const std::size_t num(dataset.size()), dim(dataset.front().size());
    Eigen::MatrixXd mat(num, dim);
    scl::convertStlToEigen(dim, dataset, mat);
    
    
    Eigen::MatrixXd covariance;
    scl::calcCovariance(mat, covariance);
    PRINT_MAT(covariance);
    

    Eigen::VectorXd x(mat.row(0));
    Eigen::VectorXd mean(mat.colwise().mean());
    PRINT_MAT(x);
    PRINT_MAT(mean);
        

    for (std::size_t i = 0; i < num; ++i)
    {
        double pdf = scl::calcPdf(mat.row(i), mean, covariance);
        std::cout << " " << log(pdf);
    }
    std::cout << std::endl;


    double log_likelihood = scl::calcLogLikelihood(mat);
    std::cout << "------------\nresult\n  " << log_likelihood << std::endl; 


    double likelihood = scl::calcLikelihood(mat);
    std::cout << "------------\nresult\n  " << likelihood << std::endl;
    std::cout << "  " << std::log(likelihood) << std::endl;

    
    return 0;
}
