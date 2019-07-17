/**
 * @file XMeans.hpp
 */

#ifndef SCL_X_MEANS_HPP
#define SCL_X_MEANS_HPP

#include <scl/clustering/KMeans.hpp>
#include <scl/util/Statistics.hpp>
#include <scl/util/EigenUtil.hpp>
#include <vector>
#include <iostream>

namespace scl
{
    /** 
     * @class XMeans
     * @brief x-means clustering.
     * @par 論文
     * <a href="http://www.cs.cmu.edu/%7Edpelleg/download/xmeans.pdf">X-means: Extending K-means with Efficient Estimation of the Number of Clusters | Carnegie Mellon Univ. (2000)</a> @n
     * <a href="http://www.rd.dnc.ac.jp/%7Etunenori/doc/xmeans_euc.pdf">クラスター数を自動決定するk-meansアルゴリズムの拡張について | 大学入試センター 研究開発部 (2000)</a>
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
         * @param[in] dataset クラスタリングするデータセット
         * @param[in] init_num_clusters 初期クラスタ数
         * @param[in,out] centroids 各クラスタの重心位置 ( method が KMeans::MANUAL の時だけ[in]も使う)
         * @return クラスタリングが収束したか
         * @details DataType needs [] access operator
         */
        template<class DataType>
        bool clustering(const std::size_t dim, const std::vector<DataType> &dataset, const std::size_t init_num_clusters, std::vector< std::vector<double> > &centroids);


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

        
    private:
        /** @brief split data */
        template<class DataType>
        void recursivelySplit(const std::vector<DataType> &dataset, const std::vector<std::size_t> &indices, const std::vector<double> &centroid);


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
    bool XMeans::clustering(const std::size_t dim, const std::vector<DataType> &dataset, const std::size_t init_num_clusters, std::vector< std::vector<double> > &centroids)
    {
        // set parameter
        m_dim = dim;
        m_clusterid_to_dataids.clear();
        m_centroids.clear();

        
        // calc first k-means
        std::vector< std::vector<double> > child_centroids;
        m_kmeans.clustering(m_dim, dataset, init_num_clusters, child_centroids, m_method);
   
        
        // x-means
        const std::vector< std::vector<std::size_t> > clusters(m_kmeans.getClusters());
        for (std::size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id)
        {
            recursivelySplit(dataset, clusters.at(cluster_id), child_centroids.at(cluster_id));
        }

        
        // copy results
        centroids.clear();
        centroids.assign(m_centroids.begin(), m_centroids.end());


        return true;
    }


    const std::vector< std::vector<std::size_t> >& XMeans::getClusters() const
    {
        return m_clusterid_to_dataids;
    }


    const std::vector<std::size_t>& XMeans::getCluster(const std::size_t cluster_id) const
    {
        return m_clusterid_to_dataids.at(cluster_id);
    }


    template<class DataType>
    void XMeans::recursivelySplit(const std::vector<DataType> &dataset, const std::vector<std::size_t> &indices, const std::vector<double> &centroid)
    {
        if ( indices.size() < 5 )
        {
            m_clusterid_to_dataids.push_back(indices);
            m_centroids.push_back(centroid);
            return;
        }

        
        // current data set
        std::vector<DataType> current_dataset;
        for (std::size_t i = 0; i < indices.size(); ++i)
        {
            current_dataset.push_back( dataset[ indices[i] ] );
        }
        
        
        // split
        std::vector< std::vector<double> > current_centroids;
        if ( !m_kmeans.clustering(m_dim, current_dataset, 2, current_centroids, m_method) )
        {
            m_clusterid_to_dataids.push_back(indices);
            m_centroids.push_back(centroid);
            return;
        }
        const std::vector< std::vector<std::size_t> > current_clusters(m_kmeans.getClusters());
        
        
        // size check
        for (std::size_t current_cluster_id = 0; current_cluster_id < current_clusters.size(); ++current_cluster_id)
        {
            if ( current_clusters.at(current_cluster_id).size() < 5 )
            {
                m_clusterid_to_dataids.push_back(indices);
                m_centroids.push_back(centroid);
                return;
            }
        }

        
        // calc current BIC
        double p(m_dim);
        double q = p * (p + 3) * 0.5;
        double ni(indices.size());
        
        double current_bic = -2.0 * scl::normal::calcLogLikelihood( scl::toEigenMatrix(m_dim, dataset, indices), scl::toEigenVector(m_dim, centroid) ) + q * std::log(ni);
 

        // calc beta
        double beta(0.0);
        {
            double squared_distance(0.0);
            for (std::size_t i = 0; i < m_dim; ++i)
            {
                double error = current_centroids[0][i] - current_centroids[1][i];
                squared_distance += (error * error);
            }

            Eigen::MatrixXd cov0, cov1;
            scl::calcCovariance( scl::toEigenMatrix(m_dim, current_dataset, current_clusters[0]), cov0);
            scl::calcCovariance( scl::toEigenMatrix(m_dim, current_dataset, current_clusters[1]), cov1);

            beta = std::sqrt( squared_distance / (cov0.determinant() + cov1.determinant()) );
        }
  

        // calc alpha
        double alpha = 0.5 / scl::normal::cumulativeDensityFunction(beta);


        // calc split BIC
        double log_likelihood0 = scl::normal::calcLogLikelihood( scl::toEigenMatrix(m_dim, current_dataset, current_clusters[0]), scl::toEigenVector(current_centroids[0]) );
        double log_likelihood1 = scl::normal::calcLogLikelihood( scl::toEigenMatrix(m_dim, current_dataset, current_clusters[1]), scl::toEigenVector(current_centroids[1]) );
        
        double split_bic = -2.0 * ( ni * std::log(alpha) + log_likelihood0 + log_likelihood1 ) + 2.0 * q * std::log(ni);


        // compare BIC
        if ( split_bic < current_bic )
        {
            for (std::size_t split_id = 0; split_id < 2; ++split_id)
            {
                std::vector<std::size_t> split_indices(current_clusters[split_id].size());
                for (std::size_t i = 0; i < current_clusters[split_id].size(); ++i)
                {
                    std::size_t original_dataset_index = indices.at( current_clusters[split_id][i] );
                    split_indices[i] = original_dataset_index;
                }
                recursivelySplit(dataset, split_indices, current_centroids[split_id]);
            }
        }
        else
        {
            m_clusterid_to_dataids.push_back(indices);
            m_centroids.push_back(centroid);
        }
    }
    
  
}  // end of namespace scl


#endif  /* SCL_X_MEANS_HPP */
