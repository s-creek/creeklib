/**
 * @file XMeans.hpp
 */

#ifndef SCL_X_MEANS_HPP
#define SCL_X_MEANS_HPP

#include <scl/clustering/KMeans.hpp>
#include <scl/util/Statistics.hpp>
#include <scl/util/EigenUtil.hpp>
#include <vector>
#include <limits>

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
        /**
         * @enum SplittingType
         * @brief 分割手法
         */
        enum SplittingType
        {
            BIC_ORG,      /**< Bayesian information criterion (pyclusteringを参考にした手法) */
            BIC_ISHIOKA,  /**< Bayesian information criterion (石岡の手法) */
            MNDL          /**< Minimum noiseless description length */
        };

        
        /** @brief コンストラクタ */
        XMeans();

    
        /**
         * @brief k-means のパラメータ設定
         * @param[in] method クラスタ重心の初期化方法 (デフォルト k-means++)
         * @see KMeans::setParameters
         */
        void setParameters(const std::size_t max_iteration, const double tolerance, const std::size_t max_pp_trial=3, const KMeans::InitMethod method=KMeans::PLUSPLUS);


        /** @brief 分割時の評価方法を設定 */
        void setSplittingType(const SplittingType splitting_type);
        

        /**
         * @brief クラスタリング
         * @tparam DataType クラスタリングするデータの型
         * @param[in] dim DataTypeの次数
         * @param[in] dataset クラスタリングするデータセット
         * @param[in] init_num_clusters 初期クラスタ数
         * @param[in,out] centroids 各クラスタの重心位置 ( method が KMeans::MANUAL の時だけ[in]も使う)
         * @param[in] min_num 各クラスタ内の最小データ数
         * @attention DataType needs [] access operator
         */
        template<class DataType>
        void clustering(const std::size_t dim, const std::vector<DataType> &dataset, const std::size_t init_num_clusters, std::vector< std::vector<double> > &centroids, const std::size_t min_num=5);


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
        /**
         * @brief split data (x-means main algorithm)
         * @param[in] dataset クラスタリングする全データ
         * @param[in] indices 分割予定のクラスタのインデックスリスト
         * @param[in] centered 分割予定のクラスタの重心
         * @param[in] min_num クラスタ内の最小データ数 (これ以下なら分割しない)
         */
        template<class DataType>
        void recursivelySplit(const std::vector<DataType> &dataset, const std::vector<std::size_t> &indices, const std::vector<double> &centroid, const std::size_t min_num);


        /**
         * @brief Bayesian information criterion, BIC
         * @details pyclustering を参考にした @n
         * <a href="https://github.com/annoviko/pyclustering">GitHub</a>
         * @bug たまに結果がよろしくない。
         *      分割するときのスコアを0.95倍にすると割といい感じ。
         */
        template<class DataType>
        double bayesianInformationCriterion(const std::vector<DataType> &dataset, const std::vector<std::vector<std::size_t> > &indices_list, const std::vector<std::vector<double> > &centroids);

        
        /**
         * @brief Bayesian information criterion, BIC
         * @details 論文の石岡の手法 @n
         * <a href="https://web-salad.hateblo.jp/entry/2014/07/19/200347">参考ブログ</a> @n
         * <a href="https://gist.github.com/yasaichi/254a060eff56a3b3b858#file-x_means-py">↑のGist</a>
         * @bug あまり結果がよろしくない・・・移植ミス？
         */
        template<class DataType>
        double bayesianInformationCriterionIshioka(const std::vector<DataType> &dataset, const std::vector<std::vector<std::size_t> > &indices_list, const std::vector<std::vector<double> > &centroids);


        /**
         * @brief Minimum noiseless description length
         * @todo 実装
         */
        double minimumNoiselessDescriptionLength();

        
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


        /** @brief 分割時の評価方法 */
        SplittingType m_splitting_type;
        
    };  // end of x-means class




    //------------------------------------------------------------------
    // 実装部
    //------------------------------------------------------------------

    XMeans::XMeans()
        : m_method(KMeans::PLUSPLUS),
          m_splitting_type(XMeans::BIC_ORG)
    {
    }
    
    
    void XMeans::setParameters(const std::size_t max_iteration, const double tolerance, const std::size_t max_pp_trial, const KMeans::InitMethod method)
    {
        m_kmeans.setParameters(max_iteration, tolerance, max_pp_trial);
        m_method = method;
    }


    void XMeans::setSplittingType(const SplittingType splitting_type)
    {
        m_splitting_type = splitting_type;
    }
    
    
    template<class DataType>
    void XMeans::clustering(const std::size_t dim, const std::vector<DataType> &dataset, const std::size_t init_num_clusters, std::vector< std::vector<double> > &centroids, const std::size_t min_num)
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
            recursivelySplit(dataset, clusters.at(cluster_id), child_centroids.at(cluster_id), min_num);
        }

        
        // copy results
        centroids.clear();
        centroids.assign(m_centroids.begin(), m_centroids.end());
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
    void XMeans::recursivelySplit(const std::vector<DataType> &dataset, const std::vector<std::size_t> &indices, const std::vector<double> &centroid, const std::size_t min_num)
    {
        if ( indices.size() < min_num )
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
            if ( current_clusters.at(current_cluster_id).size() < min_num )
            {
                m_clusterid_to_dataids.push_back(indices);
                m_centroids.push_back(centroid);
                return;
            }
        }


        // calc score
        double current_score(0.0), split_score(0.0);
        switch (m_splitting_type)
        {
        case XMeans::BIC_ISHIOKA:
        {
            current_score = bayesianInformationCriterionIshioka(dataset, std::vector<std::vector<std::size_t> >(1, indices), std::vector<std::vector<double> >(1, centroid));
            split_score = bayesianInformationCriterionIshioka(current_dataset, current_clusters, current_centroids);
            break;
        }
        case XMeans::MNDL:
        case XMeans::BIC_ORG:
        {
            current_score = bayesianInformationCriterion(dataset, std::vector<std::vector<std::size_t> >(1, indices), std::vector<std::vector<double> >(1, centroid));
            split_score = bayesianInformationCriterion(current_dataset, current_clusters, current_centroids);
            split_score *= 0.95;  // todo check
        }
        }

        
        // compare BIC
        if ( split_score < current_score )
        {
            for (std::size_t split_id = 0; split_id < 2; ++split_id)
            {
                std::vector<std::size_t> split_indices(current_clusters[split_id].size());
                for (std::size_t i = 0; i < current_clusters[split_id].size(); ++i)
                {
                    std::size_t original_dataset_index = indices.at( current_clusters[split_id][i] );
                    split_indices[i] = original_dataset_index;
                }
                recursivelySplit(dataset, split_indices, current_centroids[split_id], min_num);
            }
        }
        else
        {
            m_clusterid_to_dataids.push_back(indices);
            m_centroids.push_back(centroid);
        }
    }


    template<class DataType>
    double XMeans::bayesianInformationCriterion(const std::vector<DataType> &dataset, const std::vector<std::vector<std::size_t> > &indices_list, const std::vector<std::vector<double> > &centroids)
    {
        double bic( std::numeric_limits<double>::max() );
        
        /* 計算に使うので先にdoubleにキャストしておく */
        double dim(m_dim);
        double K(indices_list.size());
        double N(0);
        double squared_sigma(0.0);

        // calc variance
        for (std::size_t cluster_id = 0; cluster_id < indices_list.size(); ++cluster_id)
        {
            const std::vector<double> centroid(centroids.at(cluster_id));
            for (std::size_t data_index = 0; data_index < indices_list.at(cluster_id).size(); ++data_index)
            {
                DataType data = dataset.at(indices_list.at(cluster_id).at(data_index));
                squared_sigma += m_kmeans.calcSquaredDistance(data, centroid);
            }

            N += static_cast<double>(indices_list.at(cluster_id).size());
        }

        if ( N - K > 0 )
        {
            squared_sigma /= (N - K);
            double p = (K - 1) + dim * K + 1;

            // in case of the same points, sigma_sqrt can be zero
            double sigma_multiplier(0.0);
            if (squared_sigma <= 0.0)
            {
                sigma_multiplier = -1.0 * std::numeric_limits<double>::max();
            }
            else
            {
                sigma_multiplier = dim * 0.5 * std::log(squared_sigma);
            }

            // splitting criterion
            bic = 0.0;
            for (std::size_t cluster_id = 0; cluster_id < indices_list.size(); ++cluster_id)
            {
                double n = static_cast<double>(indices_list.at(cluster_id).size());
                double L = n * std::log(n) - n * std::log(N) - n * 0.5 * std::log(2.0 * M_PI) - n * sigma_multiplier - (n - K) * 0.5;
                bic += p * 0.5 * std::log(N) - L;
            }
        }

        return bic;
    }

    
    template<class DataType>
    double XMeans::bayesianInformationCriterionIshioka(const std::vector<DataType> &dataset, const std::vector<std::vector<std::size_t> > &indices_list, const std::vector<std::vector<double> > &centroids)
    {
        double bic(0.0);
        
        /* 計算に使うので先にdoubleにキャストしておく */
        double p(m_dim);
        double q = p * (p + 3) * 0.5;
        double K(indices_list.size());
        double N(0);

        // calc data size
        for (std::size_t cluster_id = 0; cluster_id < indices_list.size(); ++cluster_id)
        {
            N += static_cast<double>(indices_list.at(cluster_id).size());
        }

        // calc BIC
        for (std::size_t cluster_id = 0; cluster_id < indices_list.size(); ++cluster_id)
        {
            double log_likelihood = scl::normal::calcLogLikelihood( scl::toEigenMatrix(m_dim, dataset, indices_list.at(cluster_id)), scl::toEigenVector(m_dim, centroids.at(cluster_id)) );
            bic += -2.0 * log_likelihood + q * std::log(N);
        }

        // split
        if (K == 2)
        {
            double squared_distance = m_kmeans.calcSquaredDistance(centroids[0], centroids[1]);
            Eigen::MatrixXd cov0, cov1;
            scl::calcCovariance( scl::toEigenMatrix(m_dim, dataset, indices_list[0]), cov0);
            scl::calcCovariance( scl::toEigenMatrix(m_dim, dataset, indices_list[1]), cov1);

            double beta = std::sqrt( squared_distance / (cov0.determinant() + cov1.determinant()) );
            double alpha = 0.5 / scl::normal::cumulativeDensityFunction(beta);

            bic += -2.0 * N * std::log(alpha);
        }
        
        return bic;
    }


    double XMeans::minimumNoiselessDescriptionLength()
    {
        return 0;
    }

    
}  // end of namespace scl


#endif  /* SCL_X_MEANS_HPP */
