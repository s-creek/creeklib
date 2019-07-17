#include <scl/clustering/XMeans.hpp>
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

    
    // convert data
    const std::size_t num(dataset.size()), dim(dataset.front().size());
    Eigen::MatrixXd mat(scl::toEigenMatrix(dim, dataset));

    
    return 0;
}
