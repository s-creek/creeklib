/**
 * @file XMeans.hpp
 */

#ifndef SCL_X_MEANS_HPP
#define SCL_X_MEANS_HPP

#include <scl/clustering/KMeans.hpp>
#include <scl/util/Statistics.hpp>
#include <vector>

namespace scl
{
    /** 
     * @class XMeans
     * @brief x-means clustering.
     * @details 参考サイト @n
     * <a href="https://qiita.com/deaikei/items/8615362d320c76e2ce0b">qiita</a>
     */
    class XMeans
    {
    public:
        /** @brief コンストラクタ */
        XMeans();

    
        /**
         * @brief k-means のパラメータ設定
         * @param[in] method クラスタ重心の初期化方法 (デフォルト k-means++)
         * @see KMeans::setParameters
         */
        void setParameters(const std::size_t max_iteration, const double tolerance, const std::size_t max_pp_trial=3, const KMeans::InitMethod method=KMeans::PLUSPLUS);


        /**
         * @brief クラスタリング
         * @tparam DataType クラスタリングするデータの型
         * @param[in] dim DataTypeの次数
         * @param[in] data_set クラスタリングするデータセット
         * @param[in] init_num_clusters 初期クラスタ数
         * @param[in,out] centroids 各クラスタの重心位置 ( method が KMeans::MANUAL の時だけ[in]も使う)
         * @return クラスタリングが収束したか
         * @details DataType needs [] access operator
         */
        template<class DataType>
        bool clustering(const std::size_t dim, const std::vector<DataType> &data_set, const std::size_t init_num_clusters, std::vector< std::vector<double> > &centroids);

    
    private:
        /** @brief split data */
        template<class DataType>
        void recursivelySplit(const std::vector<DataType> &data_set, const std::vector<std::size_t> &indices, const std::vector<double> &centroid);


        /** @brief クラスタリングするデータの次数 */
        std::size_t m_dim;


        /** @brief x-means で確定したクラスタ情報 */
        std::vector< std::vector<std::size_t> > m_clusterid_to_dataids;


        /** @brief x-means で確定したクラスタの各重心 */
        std::vector< std::vector<double> > m_centroids;
                
        
        /** @brief k-means class */
        scl::KMeans m_kmeans;

        
        /** @brief k-means method */
        KMeans::InitMethod m_method;

        
    };  // end of x-means class




    //------------------------------------------------------------------
    // 実装部
    //------------------------------------------------------------------

    XMeans::XMeans()
        : m_method(KMeans::PLUSPLUS)
    {
    }
    
    
    void XMeans::setParameters(const std::size_t max_iteration, const double tolerance, const std::size_t max_pp_trial, const KMeans::InitMethod method)
    {
        m_kmeans.setParameters(max_iteration, tolerance, max_pp_trial);
        m_method = method;
    }
    
    
    template<class DataType>
    bool XMeans::clustering(const std::size_t dim, const std::vector<DataType> &data_set, const std::size_t init_num_clusters, std::vector< std::vector<double> > &centroids)
    {
        // set parameter
        m_dim = dim;
        m_clusterid_to_dataids.clear();
        m_centroids.clear();

        // calc first k-means
        std::vector< std::vector<double> > child_centroids;
        m_kmeans.clustering(m_dim, data_set, init_num_clusters, child_centroids, m_method);

        // x-means
        const std::vector< std::vector<std::size_t> > clusters(m_kmeans.getClusters());
        for (std::size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id)
        {
            recursivelySplit(data_set, clusters.at(cluster_id), child_centroids.at(cluster_id));
        }
    }


    template<class DataType>
    void XMeans::recursivelySplit(const std::vector<DataType> &data_set, const std::vector<std::size_t> &indices, const std::vector<double> &centroid)
    {
        if ( indices.size() < 3 )
        {
            m_clusterid_to_dataids.push_back(indices);
            m_centroids.push_back(centroid);
        }
    }
    
  
}  // end of namespace scl


#endif  /* SCL_X_MEANS_HPP */
