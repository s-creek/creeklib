#include <scl/clustering/KMeans.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

int main ()
{
    // file open
    std::ifstream file("sample_data.log");
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
    

    // set k-means parameter (from sample dataset)
    std::size_t dim(dataset.front().size()), cluster_size(6), max_iter(100), max_pp_trial(5);
    double tolerance(0.01);
    std::vector< std::vector<double> > centroids;
    
    
    // k-means
    scl::KMeans kmeans;
    kmeans.setParameters(max_iter, tolerance, max_pp_trial);
    if ( kmeans.clustering(dim, dataset, cluster_size, centroids, scl::KMeans::PLUSPLUS) )
    {
        std::cout << "success" << std::endl;

        // show cluster data
        std::vector< std::vector<std::size_t> > clusters( kmeans.getClusters() );
        for (std::size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id)
        {
            std::cout << " " << clusters.at(cluster_id).size();
        }
        std::cout << std::endl;
    }
    else
    {
        std::cout << "failure" << std::endl;
    }
    
    
    return 0;
}
