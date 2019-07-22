#include <sstream>
#include <fstream>
#include <vector>
#include <array>
#include <limits>
#include <functional>
#include <iostream>


class SampleClass
{
public:
    SampleClass() {}

    // template<class DataType, class Labelling>
    // void clustering(const std::vector<DataType> &dataset, const std::vector< std::vector<double> > &centroids, const Labelling &labeler, std::vector< std::vector<std::size_t> > &clusters)
    template<class DataType>
    void clustering(const std::vector<DataType> &dataset, const std::vector< std::vector<double> > &centroids,
                    const std::function<void(const std::vector<double>&, const std::vector< std::vector<double> >&, std::size_t&, double&)> &labeler,
                    std::vector< std::vector<std::size_t> > &clusters)
    {
        clusters.clear();
        clusters.resize(centroids.size());
        
        double score(0.0);
        for (std::size_t data_index = 0; data_index < dataset.size(); ++data_index) {
            const DataType &data = dataset.at(data_index);
            std::size_t nearest_cluster_index(0);
            double squared_distance(0);
            labeler(data, centroids, nearest_cluster_index, squared_distance);
            
            clusters.at(nearest_cluster_index).push_back(data_index);
            score += squared_distance;
        }
    }
};


void KMeansLabelerFunc (const std::vector<double> &data, const std::vector< std::vector<double> > &centroids, std::size_t &index, double &score) {
    score = std::numeric_limits<double>::max();
    index = 0;
    const std::size_t dim(data.size());
    for (std::size_t centroid_index = 0; centroid_index < centroids.size(); ++centroid_index) {
        double sd(0.0);
        for (std::size_t value_index = 0; value_index < dim; ++ value_index) {
            double error = static_cast<double>(data[value_index]) - centroids.at(centroid_index).at(value_index);
            sd += (error * error);
        }
        if ( sd < score) {
            index = centroid_index;
            score = sd;
        }
    }
}


int main (int argc, char **argv)
{
    // file open
    std::string file_name("./log/sample_data.log");
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


    //
    // test
    //
    SampleClass sc;
    std::vector< std::vector<double> > centroids = { {-10, 0}, {20, 10}, {5, 30}, {-40, 10}, {-20, -30}, {15, -20} };
    std::vector< std::vector<std::size_t> > clusters;
    

    //
    // lamda
    //
    sc.clustering(dataset,
                  centroids,
                  [] (const std::vector<double> &data, const std::vector< std::vector<double> > &centroids, std::size_t &index, double &score) {
                      score = std::numeric_limits<double>::max();
                      index = 0;
                      const std::size_t dim(data.size());
                      for (std::size_t centroid_index = 0; centroid_index < centroids.size(); ++centroid_index) {
                          double sd(0.0);
                          for (std::size_t value_index = 0; value_index < dim; ++ value_index) {
                              double error = static_cast<double>(data[value_index]) - centroids.at(centroid_index).at(value_index);
                              sd += (error * error);
                          }
                          if ( sd < score) {
                              index = centroid_index;
                              score = sd;
                          }
                      }
                  },
                  clusters
                  );

    for (std::size_t i = 0; i < clusters.size(); ++ i) {
        std::cout << "  " << clusters.at(i).size();
    }
    std::cout << std::endl;


    //
    // struct
    //
    struct KMeansLabeler
    {
        void operator() (const std::vector<double> &data, const std::vector< std::vector<double> > &centroids, std::size_t &index, double &score) const {
            score = std::numeric_limits<double>::max();
            index = 0;
            const std::size_t dim(data.size());
            for (std::size_t centroid_index = 0; centroid_index < centroids.size(); ++centroid_index) {
                double sd(0.0);
                for (std::size_t value_index = 0; value_index < dim; ++ value_index) {
                    double error = static_cast<double>(data[value_index]) - centroids.at(centroid_index).at(value_index);
                    sd += (error * error);
                }
                if ( sd < score) {
                    index = centroid_index;
                    score = sd;
                }
            }
        }
    };

    clusters.clear();
    sc.clustering(dataset,
                  centroids,
                  KMeansLabeler(),
                  clusters
                  );
    
    for (std::size_t i = 0; i < clusters.size(); ++ i) {
        std::cout << "  " << clusters.at(i).size();
    }
    std::cout << std::endl;


    //
    // function
    //
    clusters.clear();
    sc.clustering(dataset,
                  centroids,
                  KMeansLabelerFunc,
                  clusters
                  );
    
    for (std::size_t i = 0; i < clusters.size(); ++ i) {
        std::cout << "  " << clusters.at(i).size();
    }
    std::cout << std::endl;

    
    return 0;
}
