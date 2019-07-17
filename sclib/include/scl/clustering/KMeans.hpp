/**
 * @file KMeans.hpp
 * @brief This class implements the k-means clustering algorithm.
 */

#ifndef SCL_K_MEANS_HPP
#define SCL_K_MEANS_HPP

#include <vector>
#include <numeric>    // iota
#include <algorithm>  // shuffle
#include <random>     // random
#include <limits>     // limit

#include <cassert>    // assert
#include <iostream>   // debug

namespace scl
{
    /** 
     * @class KMeans
     * @brief k-means clustering.
     * @details 参考サイト @n
     * <a href="https://www.slideshare.net/tomo_otamot/kmeansk">slideshare 1</a> @n
     * <a href="https://www.slideshare.net/oscillograph/6-51143088">slideshare 2</a> @n
     * <a href="http://wakaba-technica.sakura.ne.jp/library/clustering_kmean.html">k-means++</a>
     */
    class KMeans
    {
    public:
        /**
         * @enum InitMethod
         * @brief クラスタ重心の初期化方法 
         */
        enum InitMethod
        {
            RANDOM,   /**< ランダムにクラスタリングして重心計算 */
            UNIFORM,  /**< 頭から順番にクラスタリングして重心計算 */
            PLUSPLUS, /**< k-means++ */
            MANUAL    /**< ユーザの入力をそのまま使う */
        };


        /** @brief コンストラクタ */
        KMeans();
        
        
        /** @brief デストラクタ */
        virtual ~KMeans();


        /**
         * @brief クラスタリング時のパラメータ設定
         * @param[in] max_iteration 最大試行回数
         * @param[in] tolerance 収束判定閾値
         * @param[in] max_pp_trial k-means++で初期化するときの最大試行回数 (max_iteration とは別)
         */
        void setParameters(const std::size_t max_iteration, const double tolerance, const std::size_t max_pp_trial=3);


        /**
         * @brief クラスタリング
         * @tparam DataType クラスタリングするデータの型
         * @param[in] dim DataTypeの次数
         * @param[in] dataset クラスタリングするデータセット
         * @param[in] num_clusters クラスタ数
         * @param[in,out] centroids 各クラスタの重心位置 ( method が  KMeans::MANUAL の時だけ[in]も使う)
         * @param[in] method クラスタ重心の初期化方法 (デフォルト k-means++)
         * @return クラスタリングが収束したか (重心の更新値が KMeans::m_tolerance 以下に収まっているか)
         * @details DataType needs [] access operator
         */
        template<class DataType>
        bool clustering(const std::size_t dim, const std::vector<DataType> &dataset, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids, const InitMethod method=PLUSPLUS);


        /**
         * @brief クラスタリング
         * @see KMeans::clustering
         */
        template<class DataType>
        bool clustering(const std::size_t dim, const std::vector<DataType> &dataset, const std::size_t num_clusters, const InitMethod method=PLUSPLUS);


        /** 
         * @brief 全クラスタの情報取得
         * @see KMeans::m_clusterid_to_dataids
         */
        const std::vector< std::vector<std::size_t> > & getClusters() const;


        /**
         * @brief 指定したクラスタの情報取得
         * @see KMeans::m_clusterid_to_dataids
         */
        const std::vector<std::size_t> & getCluster(const std::size_t cluster_id) const;
        
        
    protected: 
        /**
         * @brief 乱数によるクラスタ重心の初期化
         * @tparam DataType クラスタリングするデータの型
         * @param[in] dataset クラスタリングするデータセット
         * @param[in] num_clusters クラスタ数
         * @param[out] centroids 各クラスタの重心位置
         */
        template<class DataType>
        void initCentroidsRandom(const std::vector<DataType> &dataset, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids);


        /**
         * @brief 頭から順番にクラスタリングして重心初期化
         * @see Kmeans::initCentroidsRandom
         */
        template<class DataType>
        void initCentroidsUniform(const std::vector<DataType> &dataset, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids);


        /**
         * @brief k-means++によるクラスタ重心の初期化
         * @see Kmeans::initCentroidsRandom
         */
        template<class DataType>
        void initCentroidsPlusplus(const std::vector<DataType> &dataset, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids);


        /**
         * @brief データと重心の距離の2乗
         * @param[in] point データセットの中の一点
         * @param[in] centroid 重心位置
         * @return point と centroid の距離の2乗
         */
        template<class DataTypeA, class DataTypeB>
        double calcSquaredDistance(const DataTypeA &point_a, const DataTypeB &point_b);

    
        /**
         * @brief ラベルの更新
         * @tparam DataType クラスタリングするデータの型
         * @param[in] dataset クラスタリングするデータセット
         * @param[out] centroids 各クラスタの重心位置
         * @return 評価値
         */
        template<class DataType>
        double updateLabel(const std::vector<DataType> &dataset, const std::vector< std::vector<double> > &centroids);
    

        /**
         * @brief クラスタ重心の計算
         * @tparam DataType クラスタリングするデータの型
         * @param[in] dataset クラスタリングするデータセット
         * @param[out] centroids 各クラスタの重心位置
         */
        template<class DataType>
        void calcCentroids(const std::vector<DataType> &dataset, std::vector< std::vector<double> > &centroids);
        
        
        /**
         * @brief クラスタID(=index)ごとにデータIDを割り振った配列
         * @par example
         * data_indices = this[cluster_index] @n
         * data_index = this[cluster_index][array_index]
         */
        std::vector< std::vector<std::size_t> > m_clusterid_to_dataids;

    
        /** @brief 最大試行回数 */
        std::size_t m_max_iteration;

    
        /** @brief 収束判定閾値 */
        double m_tolerance;


        /** @brief k-means++で初期化するときの最大試行回数 */
        std::size_t m_max_pp_trial;


        /** @brief クラスタリングするデータの次数 */
        std::size_t m_dim;
        
        
    };  // end of k-means class



    
    //------------------------------------------------------------------
    // 実装部
    //------------------------------------------------------------------

    KMeans::KMeans()
        : m_max_iteration(10),
          m_tolerance(0.1),
          m_max_pp_trial(3),
          m_dim(0)
    {
    }


    KMeans::~KMeans()
    {
    }


    void KMeans::setParameters(const std::size_t max_iteration, const double tolerance, const std::size_t max_pp_trial)
    {
        m_max_iteration = max_iteration;
        m_tolerance = tolerance;
        m_max_pp_trial = max_pp_trial;
    }


    template<class DataType>
    bool KMeans::clustering(const std::size_t dim, const std::vector<DataType> &dataset, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids, const InitMethod method)
    {
        // size check
        if (dim == 0 || dataset.empty() || num_clusters == 0)
        {
            return false;
        }
        m_dim = dim;


        // クラスタ重心の初期化 //
        switch (method)
        {
        case KMeans::RANDOM:
        {
            initCentroidsRandom(dataset, num_clusters, centroids);
            break;
        }
        case KMeans::UNIFORM:
        {
            initCentroidsUniform(dataset, num_clusters, centroids);
            break;
        }
        case KMeans::PLUSPLUS:
        {
            initCentroidsPlusplus(dataset, num_clusters, centroids);
            break;
        }
        case KMeans::MANUAL:
        {
            if (num_clusters != centroids.size())
            {
                return false;
            }
        }
        } // end of switch method


        // k-means
        double pre_cost(-m_tolerance);  // 最初の一回で収束しないように //
        bool is_converged(false);
        for (std::size_t iteration = 0; iteration < m_max_iteration; ++iteration)
        {
            // update label
            double cost = updateLabel(dataset, centroids);

             // update centroids
            calcCentroids(dataset, centroids);

            // check converged
            double error(cost - pre_cost);
            if (error < m_tolerance)
            {
                is_converged = true;
                break;
            }
            pre_cost = cost;
        }
        return is_converged;
    }


    template<class DataType>
    bool KMeans::clustering(const std::size_t dim, const std::vector<DataType> &dataset, const std::size_t num_clusters, const InitMethod method)
    {
        std::vector< std::vector<double> > centroids;
        return clustering(dim, dataset, num_clusters, centroids, method);
    }


    const std::vector< std::vector<std::size_t> >& KMeans::getClusters() const
    {
        return m_clusterid_to_dataids;
    }


    const std::vector<std::size_t>& KMeans::getCluster(const std::size_t cluster_id) const
    {
        return m_clusterid_to_dataids.at(cluster_id);
    }


    template<class DataType>
    void KMeans::initCentroidsRandom(const std::vector<DataType> &dataset, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids)
    {
        // データセットのインデックスリストを作成 //
        std::vector<std::size_t> shuffle_indices(dataset.size());
        std::iota(shuffle_indices.begin(), shuffle_indices.end(), 0);

         // シャッフル //
        std::random_device seed_gen;
        std::mt19937 engine(seed_gen());
        std::shuffle(shuffle_indices.begin(), shuffle_indices.end(), engine);
    
        // init label
        m_clusterid_to_dataids.resize(num_clusters);
        for (std::size_t shuffle_index = 0; shuffle_index < shuffle_indices.size(); ++shuffle_index)
        {
            std::size_t data_index = shuffle_indices.at(shuffle_index);
            std::size_t cluster_index = data_index % num_clusters;
            m_clusterid_to_dataids[cluster_index].push_back(data_index);
        }
    
        // init centroids
        centroids.resize(num_clusters, std::vector<double>(m_dim, 0.0));
        calcCentroids(dataset, centroids);
    }


    template<class DataType>
    void KMeans::initCentroidsUniform(const std::vector<DataType> &dataset, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids)
    {
        // init label
        m_clusterid_to_dataids.resize(num_clusters);
        for (std::size_t data_index = 0; data_index < dataset.size(); ++data_index)
        {
            std::size_t cluster_index = data_index % num_clusters;
            m_clusterid_to_dataids[cluster_index].push_back(data_index);
        }

        // init centroids
        centroids.resize(num_clusters, std::vector<double>(m_dim, 0.0));
        calcCentroids(dataset, centroids);
    }


    template<class DataType>
    void KMeans::initCentroidsPlusplus(const std::vector<DataType> &dataset, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids)
    {
        // init data
        const std::size_t dim(m_dim);
        centroids.clear();
        centroids.reserve(num_clusters);

        // 乱数器の生成 //
        std::random_device seed_gen;
        std::mt19937 engine(seed_gen());
        std::uniform_int_distribution<std::size_t> index_distribution(0, dataset.size()-1);  // [min, max] 最大値以下 //
        std::uniform_real_distribution<double> threshold_distribution(0.0, 1.0);              // [min, max) 最大値未満 //

        // 1個目のクラスタ重心位置をデータからランダムに選択 //
        {
            std::size_t random_index = index_distribution(engine);
            const DataType &target(dataset.at(random_index));
            std::vector<double> centroid(dim, 0.0);
            for (std::size_t value_index = 0; value_index < dim; ++value_index)
            {
                centroid[value_index] = static_cast<double>(target[value_index]);
            }
            centroids.push_back(centroid);
        }

        // initialize list data
        std::vector<double> distance_list(dataset.size());
        double sum_squared_distance(0.0);
        for (std::size_t data_index = 0; data_index < dataset.size(); ++data_index)
        {
            distance_list[data_index] = calcSquaredDistance(dataset.at(data_index), centroids.back());
            sum_squared_distance += distance_list[data_index];
        }

        // k-means++
        for (std::size_t cluster_index = 1; cluster_index < num_clusters; ++cluster_index)
        {
            // 次の重心位置を決める試行計算 //
            std::vector<double> next_centroid(centroids.back());
            std::vector<double> best_distance_list(distance_list);
            double best_sum_squared_distance(sum_squared_distance);
    
            for (std::size_t trial = 0; trial < m_max_pp_trial; ++trial)
            {
                std::vector<double> proposed_centroid(centroids.back());
                std::vector<double> proposed_distance_list(distance_list);
                double proposed_sum_squared_distance(0.0);

                // 次の重心位置を決める閾値を設定 //
                double threshold = sum_squared_distance * threshold_distribution(engine);

                for (std::size_t data_index = 0; data_index < dataset.size(); ++data_index)
                {
                    threshold -= distance_list[data_index];
                    if (threshold < 0)
                    {
                        // new centroid
                        const DataType &target(dataset.at(data_index));
                        for (std::size_t value_index = 0; value_index < dim; ++value_index)
                        {
                            proposed_centroid[value_index] = static_cast<double>(target[value_index]);
                        }
                        break;
                    }
                }

                // proposed_centroid 時の評価値計算 //
                for (std::size_t data_index = 0; data_index < dataset.size(); ++data_index)
                {
                    double squared_distance = calcSquaredDistance(dataset.at(data_index), proposed_centroid);
                    if ( squared_distance < distance_list[data_index] )
                    {
                        proposed_distance_list[data_index] = squared_distance;
                    }
                    proposed_sum_squared_distance += proposed_distance_list[data_index];
                }

                // update best data
                if (proposed_sum_squared_distance < best_sum_squared_distance)
                {
                    next_centroid.swap(proposed_centroid);
                    best_distance_list.swap(proposed_distance_list);
                    best_sum_squared_distance = proposed_sum_squared_distance;
                }            
            
            } // end of each trial
        
            // update for next step
            centroids.push_back(next_centroid);
            distance_list.swap(best_distance_list);
            sum_squared_distance = best_sum_squared_distance;
        
        } // end of each cluster
    }


    template<class DataTypeA, class DataTypeB>
    double KMeans::calcSquaredDistance(const DataTypeA &point_a, const DataTypeB &point_b)
    {
        double squared_distance(0.0);

        for (std::size_t value_index = 0; value_index < m_dim; ++value_index)
        {
            double value_error = static_cast<double>(point_a[value_index]) - static_cast<double>(point_b[value_index]);
            squared_distance += (value_error * value_error);
        }
        
        return squared_distance;
    }


    template<class DataType>
    double KMeans::updateLabel(const std::vector<DataType> &dataset, const std::vector< std::vector<double> > &centroids)
    {
        // set size data
        const std::size_t dim(m_dim);
    
        // reset label
        m_clusterid_to_dataids.clear();
        m_clusterid_to_dataids.resize(centroids.size());

        // for each data
        double cost(0);
        for (std::size_t data_index = 0; data_index < dataset.size(); ++data_index)
        {
            // set target data
            double min_squared_distance(std::numeric_limits<double>::max());
            std::size_t nearest_cluster_index(0);

            // search nearest cluster (centroid)
            for (std::size_t cluster_index = 0; cluster_index < centroids.size(); ++cluster_index)
            {
                // calc squared distance
                double new_squared_distance = calcSquaredDistance(dataset.at(data_index), centroids.at(cluster_index));

                // update nearest cluster (centroid)
                if (new_squared_distance < min_squared_distance)
                {
                    min_squared_distance = new_squared_distance;
                    nearest_cluster_index = cluster_index;
                }
            }

            // asign data to nearest cluster
            m_clusterid_to_dataids.at(nearest_cluster_index).push_back(data_index);
            cost += min_squared_distance;
        }
        return cost;
    }
    

    template<class DataType>
    void KMeans::calcCentroids(const std::vector<DataType> &dataset, std::vector< std::vector<double> > &centroids)
    {
        // set size data
        const std::size_t cluster_size(m_clusterid_to_dataids.size());
        const std::size_t dim(m_dim);

        // for each cluster
        for (std::size_t cluster_index = 0; cluster_index < cluster_size; ++cluster_index)
        {
            // num data in this cluster
            std::size_t num_data = m_clusterid_to_dataids.at(cluster_index).size();

            // no data (error)
            if (num_data == 0)
            {
                // set zero
                std::vector<double> &centroid = centroids.at(cluster_index);
                for (std::size_t value_index = 0; value_index < dim; ++value_index)
                {
                    centroid[value_index] = 0;
                }
            }
            else
            {
                // sum value
                double sum[dim] = { 0 };

                // calc sum
                for (std::size_t i = 0; i < num_data; ++i)
                {
                    std::size_t data_index = m_clusterid_to_dataids.at(cluster_index).at(i);
                    const DataType &target(dataset.at(data_index));
                    
                    for (std::size_t value_index = 0; value_index < dim; ++value_index)
                    {
                        sum[value_index] += static_cast<double>(target[value_index]);
                    }
                }
                
                // calc centroid
                std::vector<double> &centroid(centroids.at(cluster_index));
                double num = static_cast<double>(num_data);
                
                for (std::size_t value_index = 0; value_index < dim; ++value_index)
                {
                    centroid[value_index] = sum[value_index] / num;
                }
            
            }  // end : exist points
        
        }  // end : each cluster
    
    }
    
} // end of namespace scl


#endif  /* SCL_K_MEANS_HPP */
