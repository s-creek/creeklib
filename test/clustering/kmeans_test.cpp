#include <scl/clustering/KMeans.hpp>

#include <array>
#include <chrono>
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/flann/flann.hpp>


template<class DataType>
void saveClusters(std::string filename, const std::size_t dim, const std::vector<DataType> &data_set, const std::vector< std::vector<std::size_t> > &clusters)
{
    std::ofstream log(filename);

    for (std::size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id)
    {
        for (std::size_t data_num = 0; data_num < clusters.at(cluster_id).size(); ++data_num)
        {
            std::size_t data_id = clusters.at(cluster_id).at(data_num);
            const DataType &point = data_set.at(data_id);

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
    // define point type
    using Point = std::array<double, 2>;
    
    
    // set k-means parameter
    std::size_t cluster_size(1), max_iter(100), max_pp_trial(5);
    double tolerance(0.01);
    
    
    // generate random points
    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());
    std::uniform_real_distribution<double> dist(-5.0, 5.0);
    
    std::vector<Point> points;
    {
        std::vector< std::vector<double> > centroids = { {-10, 0}, {20, 10}, {5, 30}, {-40, 10}, {-20, -30}, {15, -20} };
        cluster_size = centroids.size();
        
        for (std::size_t centroid_id = 0; centroid_id < centroids.size(); ++centroid_id)
        {
            const std::vector<double> &centroid = centroids.at(centroid_id);          
            std::vector<Point> local_points(15);
            for (std::size_t point_index = 0; point_index < local_points.size(); ++point_index)
            {
                local_points.at(point_index)[0] = dist(engine) + centroid[0];
                local_points.at(point_index)[1] = dist(engine) + centroid[1];
            }
            points.insert(points.end(), local_points.begin(), local_points.end());
        }
    }


    // save points
    {
        std::ofstream dat("../data/sample_data.log");
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            dat << points.at(i).at(0) << " " << points.at(i).at(1) << std::endl;
        }
        dat.close();
    }
    

    // for compare results
    std::vector< std::vector<std::size_t> > clusters_random(cluster_size);
    std::vector< std::vector<std::size_t> > clusters_uniform(cluster_size);
    std::vector< std::vector<std::size_t> > clusters_pp(cluster_size);
    std::vector< std::vector<std::size_t> > clusters_manual(cluster_size);
    std::vector< std::vector<std::size_t> > clusters_cv(cluster_size);
    
    
    // k-means (own method)
    {
        scl::KMeans kmeans;
        kmeans.setParameters(max_iter, tolerance, max_pp_trial);

        std::vector< std::vector<double> > centroids;

        kmeans.clustering(2, points, cluster_size, centroids, scl::KMeans::InitMethod::RANDOM);
        clusters_random = kmeans.getClusters();

        kmeans.clustering(2, points, cluster_size, centroids, scl::KMeans::InitMethod::UNIFORM);
        clusters_uniform = kmeans.getClusters();

        kmeans.clustering(2, points, cluster_size, centroids, scl::KMeans::InitMethod::PLUSPLUS);
        clusters_pp = kmeans.getClusters();

        kmeans.clustering(2, points, cluster_size, centroids, scl::KMeans::InitMethod::MANUAL);
        clusters_manual = kmeans.getClusters();
    }


    // k-means (opencv)
    {
        // convert to opencv
        cv::Mat cv_points(cv::Size(2, points.size()), CV_32F);
        
        for (unsigned int i = 0; i < points.size(); ++i)
        {
            for (unsigned int j = 0; j < 2; ++j)
            {
                cv_points.at<float>(i, j) = points.at(i).at(j);
            }
        }

        // k-means
        cv::Mat cv_clusters = cv::Mat::zeros(cv_points.rows, 1, CV_32F);
        cv::Mat cv_centers;
        cv::kmeans(cv_points,
                   (int)cluster_size,
                   cv_clusters,
                   cvTermCriteria(CV_TERMCRIT_EPS|CV_TERMCRIT_ITER, (int)max_iter, tolerance),
                   1,
                   cv::KMEANS_PP_CENTERS,
                   cv_centers);

        // save result
        for (int i = 0; i < cv_clusters.rows; ++i)
        {
            clusters_cv.at(cv_clusters.at<int>(i)).push_back(i);
        }
    }


    // check results
    {
        for (std::size_t cluster_id = 0; cluster_id < cluster_size; ++cluster_id)
        {
            std::cout << cluster_id
                      << "\t" << clusters_random.at(cluster_id).size()
                      << "\t" << clusters_uniform.at(cluster_id).size()
                      << "\t" << clusters_pp.at(cluster_id).size()
                      << "\t" << clusters_manual.at(cluster_id).size()
                      << "\t" << clusters_cv.at(cluster_id).size()
                      << std::endl;
        }

        saveClusters("../data/random.log", 2, points, clusters_random);
        saveClusters("../data/uniform.log", 2, points, clusters_uniform);
        saveClusters("../data/pp.log", 2, points, clusters_pp);
        saveClusters("../data/manual.log", 2, points, clusters_manual);
        saveClusters("../data/cv.log", 2, points, clusters_cv);
    }
    
    
    return 0;
}
