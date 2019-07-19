#include <scl/clustering/XMeans.hpp>
#include <scl/util/EigenUtil.hpp>

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <queue>

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl


template<class DataType>
void saveClusters(std::string filename, const std::size_t dim, const std::vector<DataType> &dataset, const std::vector< std::vector<std::size_t> > &clusters)
{
    std::ofstream log(filename);

    for (std::size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id)
    {
        for (std::size_t data_num = 0; data_num < clusters.at(cluster_id).size(); ++data_num)
        {
            std::size_t data_id = clusters.at(cluster_id).at(data_num);
            const DataType &point = dataset.at(data_id);

            for (std::size_t value_index = 0; value_index < dim; ++value_index)
            {
                log << " " << point[value_index];
            }
        }
        log << std::endl;
    }
    log.close();
}


int main ()
{
    // file open
    std::ifstream file("./log/sample_dataset_00.log");
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


    // x-means
    scl::XMeans xmeans;
    xmeans.setParameters(10, 0.025, 5, scl::KMeans::PLUSPLUS);
    std::vector< std::vector<double> > centroids;
    xmeans.clustering(dim, dataset, 2, centroids);

    std::cout << "cluster num : " << xmeans.getClusters().size() << std::endl;
    std::priority_queue<std::size_t, std::vector<std::size_t>, std::greater<std::size_t> > pque;
    for (std::size_t cluster_id = 0; cluster_id < xmeans.getClusters().size(); ++cluster_id)
    {
        std::cout << "  " << xmeans.getCluster(cluster_id).size();

        for (std::size_t data_index = 0; data_index < xmeans.getCluster(cluster_id).size(); ++data_index)
        {
            pque.push(xmeans.getCluster(cluster_id).at(data_index));
        }
    }
    std::cout << std::endl;


    saveClusters("./log/clustering_xmean_cpp.log", dim, dataset, xmeans.getClusters());


    // while (!pque.empty()) {
    //     std::cout << "  " << pque.top();
    //     pque.pop();
    // }
    // std::cout << std::endl;
    
    
    return 0;
}
