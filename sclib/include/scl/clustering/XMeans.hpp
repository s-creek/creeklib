// -*- coding: utf-8; tab-width: 4; mode: c++ -*-

/**
 * @file XMeans.hpp
 */

#ifndef SCL_X_MEANS_HPP
#define SCL_X_MEANS_HPP

#include <scl/clustering/KMeans.hpp>
#include <scl/util/Statistics.hpp>

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
        m_kmeans.clustering(dim, data_set, init_num_clusters, centroids, m_method);
    }
  
}  // end of namespace scl


#endif  /* SCL_X_MEANS_HPP */
