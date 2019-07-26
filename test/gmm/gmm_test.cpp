#include <scl/clustering/GaussianMixtureModel.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>


template<class DataType>
void saveClusters(std::string filename, const std::size_t dim, const std::vector<DataType> &dataset, const std::vector< std::vector<std::size_t> > &clusters)
{
    std::ofstream log(filename);

    for (std::size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id)
    {
        if( clusters.at(cluster_id).empty() )
        {
            continue;
        }

        
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

    
    scl::GaussianMixtureModel *gmm = new scl::GaussianMixtureModel();
    std::vector< std::vector<double> > centroids;
    gmm->clustering(2, dataset, 4, centroids);
    return 0;

    for (std::size_t k = 0; k < gmm->getClusters().size(); ++k)
    {
        std::cout << "  " << gmm->getCluster(k).size();
    }
    std::cout << std::endl;

    saveClusters("./log/clustering_gmm_cpp.log", 2, dataset, gmm->getClusters());


    // bic test
    {
        Eigen::MatrixXd data_eigen( scl::toEigenMatrix(2, dataset) );

        std::size_t best_k(3);
        for (int i = 0; i < 1; ++i)
        {
            double best_bic = 10000;
            best_k = 3;
            for (std::size_t k = best_k; k < 8; ++k)
            {
                gmm->clustering(2, dataset, k, centroids);
                //std::cout << k << "  " << gmm->calcBIC(data_eigen) << std::endl;
                double bic = gmm->calcBIC(data_eigen);
                std::cout << k << "  " << bic << std::endl;
                // for (int a = 0; a < k; ++a)
                // {
                //     std::cout << gmm->m_mean[a][0] << ", " << gmm->m_mean[a][1] << std::endl;
                // }
                // std::cout << std::endl;
                
                if ( std::isnan(bic) )
                {
                    continue;
                }
                else if (bic < best_bic)
                {
                    best_bic = bic;
                    best_k = k;
                }
            }
            std::cout << "best cluster size : " << best_k << std::endl;
        }
        gmm->clustering(2, dataset, best_k, centroids);
        saveClusters("./log/clustering_gmm_best_cpp.log", 2, dataset, gmm->getClusters());
    }
    
    
    return 0;
}
