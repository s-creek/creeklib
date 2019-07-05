/**
 * @file Kmeans.hpp
 */

#ifndef K_MEANS_HPP
#define K_MEANS_HPP

#include <vector>
#include <numeric>    // iota
#include <algorithm>  // shuffle
#include <limits>     // limit


/**
 * @brief k-means clustering.
 * @details 参考サイト @n
 * <a href="https://www.slideshare.net/tomo_otamot/kmeansk">slideshare 1</a> @n
 * <a href="https://www.slideshare.net/oscillograph/6-51143088">slideshare 2</a>
 */
class Kmeans
{
public:
    /**
     * @enum InitMethod
     * @brief クラスタ重心の初期化方法 
     */
    enum InitMethod
    {
        RANDOM,   /**< ランダムにクラスタリング */
        SERIAL,   /**< 頭から順番にクラスタリング */
        PLUSPLUS  /**< k-means++ */
    };

    
    /** @brief コンストラクタ */
    Kmeans();

    
    /**
     * @brief クラスタリング時のパラメータ設定
     * @param[in] max_iteration 最大試行回数
     * @param[in] tolerance 収束判定閾値
     */
    void setParam(const std::size_t max_iteration, const double tolerance);

    
    /**
     * @brief クラスタリング
     * @param[in] data クラスタリングする元データ
     * @param[in] num_clusters クラスタ数
     * @param[out] centroids 各クラスタの重心位置
     * @param[in] method クラスタ重心の初期化方法 (デフォルト k-means++)
     * @details DataType needs size() function & [] operator
     */
    template<class DataType>
    bool clustering(const std::vector<DataType> &data, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids, const InitMethod method=PLUSPLUS);

    template<class DataType>
    bool clustering(const std::vector<DataType> &data, const std::size_t num_clusters, const InitMethod method=PLUSPLUS) {
        std::vector< std::vector<double> > centroids;
        return this->clustering(data, num_clusters, method);
    }

    
protected:
    /**
     * @brief 乱数によるクラスタ重心の初期化
     * @param[in] data クラスタリングする元データ
     * @param[in] num_clusters クラスタ数
     * @param[out] centroids 各クラスタの重心位置
     */
    template<class DataType>
    void initCentroidsRandom(const std::vector<DataType> &data, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids);


    /**
     * @brief シリアライズによるクラスタ重心の初期化
     * @param[in] data クラスタリングする元データ
     * @param[in] num_clusters クラスタ数
     * @param[out] centroids 各クラスタの重心位置
     */
    template<class DataType>
    void initCentroidsSerial(const std::vector<DataType> &data, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids);


    /**
     * @brief k-means++によるクラスタ重心の初期化
     * @param[in] data クラスタリングする元データ
     * @param[in] num_clusters クラスタ数
     * @param[out] centroids 各クラスタの重心位置
     */
    template<class DataType>
    void initCentroidsPlusplus(const std::vector<DataType> &data, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids);


    /**
     * @brief クラスタ重心の計算
     * @param[in] data クラスタリングする元データ
     * @param[out] centroids 各クラスタの重心位置
     */
    template<class DataType>
    void calcCentroids(const std::vector<DataType> &data, std::vector< std::vector<double> > &centroids);
    
    
    /**
     * @brief クラスタIDごとにデータIDを割り振った配列
     * @par example
     * data_indices = this[cluster_id] @n
     * data_index = this[cluster_id][array_index]
     */
    std::vector< std::vector<std::size_t> > m_clusterid_to_dataids;

    
    /** @brief 最大試行回数 */
    std::size_t m_max_iteration;

    
    /** @brief 収束判定閾値 */
    double m_tolerance;
};




//------------------------------------------------------------------
// 実装部
//------------------------------------------------------------------


Kmeans::Kmeans()
    : m_max_iteration(10),
      m_tolerance(0.1)
{
}


void Kmeans::setParam(const std::size_t max_iteration, const double tolerance)
{
    m_max_iteration = max_iteration;
    m_tolerance = tolerance;
}


template<class DataType>
bool Kmeans::clustering(const std::vector<DataType> &data, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids, const InitMethod method)
{
    // size check
    if (data.empty() || num_clusters == 0)
    {
        return false;
    }

    
    // クラスタ重心の初期化 //
    switch (method)
    {
    case Kmeans::RANDOM:
    {
        initCentroidsRandom(data, num_clusters, centroids);
        break;
    }
    case Kmeans::SERIAL:
    {
        initCentroidsSerial(data, num_clusters, centroids);
        break;
    }
    case Kmeans::PLUSPLUS:
    {
        initCentroidsPlusplus(data, num_clusters, centroids);
    }
    } // end of switch method

    
}


template<class DataType>
void Kmeans::initCentroidsRandom(const std::vector<DataType> &data, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids)
{
}


template<class DataType>
void Kmeans::initCentroidsSerial(const std::vector<DataType> &data, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids)
{
    // reset cluster
    m_clusterid_to_dataids.clear();


    // init label
    m_clusterid_to_dataids.resize(num_clusters);
    for (std::size_t data_index = 0; data_index < data.size(); ++data_index)
    {
        std::size_t cluster_index = data_index % num_clusters;
        m_clusterid_to_dataids[cluster_index].push_back(data_index);
    }
}


template<class DataType>
void Kmeans::initCentroidsPlusplus(const std::vector<DataType> &data, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids)
{
    // data init
    centroids.clear();
    const std::size_t dim = data.front().size();

    
    // 1個目のクラスタ重心位置をデータからランダムに選択 //
    {
        // データのインデックスリストを作成 //
        std::vector<std::size_t> indices(data.size());
        std::iota(indices.begin(), indices.end(), 0);
        
        // シャッフル //
        std::random_device seed_gen;
        std::mt19937 engine(seed_gen());
        std::shuffle(indices.begin(), indices.end(), engine);
        
        // 初期クラスタ重心位置 //
        const std::size_t random_index = indices.front();
        DataType &point = data.at(random_index);
        std::vector<double> centroid(dim);
        for (std::size_t value_index = 0; value_index < dim; ++value_index)
        {
            centroid[value_index] = static_cast<double>(point[value_index]);
        }
        centroids.push_back(centroid);
    }


    // k-means++
    
}


template<class DataType>
void Kmeans::calcCentroids(const std::vector<DataType> &data, std::vector< std::vector<double> > &centroids)
{

}


#endif  /* K_MEANS_HPP */
