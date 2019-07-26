/**
 * @file GaussianMixtureModel.hpp
 */

#ifndef SCL_GAUSSIAN_MIXTURE_MODEL_HPP
#define SCL_GAUSSIAN_MIXTURE_MODEL_HPP

#include <scl/util/Statistics.hpp>
#include <scl/util/EigenUtil.hpp>
#include <scl/clustering/KMeans.hpp>
#include <vector>
#include <numeric>    // iota
#include <algorithm>  // shuffle
#include <random>     // random

#include <iostream>   // debug
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

namespace scl
{
    /**
     * @class GaussianMixtureModel
     * @brief Gaussian Mixture Model (GMM : 混合ガウスモデル)
     * @details クラスタ数 = 正規分布数 @n
     * 参考サイト @n
     * <a href="https://datachemeng.com/gaussianmixturemodel/">GMM</a> @n
     * <a href="http://nonbiri-tereka.hatenablog.com/entry/2014/07/29/100733">code</a> @n
     * <a href="https://qiita.com/BigSea/items/1949b3ceefcec4fc32ea">logsumexp</a>
     */
    class GaussianMixtureModel
    {
    public:
        /* for eigen */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** @brief コンストラクタ */
        GaussianMixtureModel();

        template<class DataType>
        bool clustering(const std::size_t dim, const std::vector<DataType> &dataset, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids);

        
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

        double calcBIC(const Eigen::MatrixXd &dataset);
                
        
    private:
        template<class DataType>
        void initialize2(const std::vector<DataType> &dataset);
        
        void initialize(const Eigen::MatrixXd &dataset);
        void expectationStep(const Eigen::MatrixXd &dataset);
        void maximizationStep(const Eigen::MatrixXd &dataset);
        
        
        /** @brief クラスタリングするデータの次数 */
        std::size_t m_dim;

        /** @brief クラスタ数 */
        std::size_t m_num_clusters;

        /** @brief 負担率 (N:データ数) x (k:クラスタ数) */
        std::vector< std::vector<double> > m_gamma;

        /** @brief 各クラスタの正規化時の平均 (k x dim) */
        std::vector< Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > m_mean;

        /** @brief 各クラスタの分散共分散 (k x dim x dim) */
        std::vector< Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > m_covariance;

        /** @brief 各クラスタの混合係数 (各正規分布の重み) */
        std::vector<double> m_pi;

        /**
         * @brief クラスタID(=index)ごとにデータIDを割り振った配列
         * @par example
         * data_indices = this[cluster_index] @n
         * data_index = this[cluster_index][array_index]
         */
        std::vector< std::vector<std::size_t> > m_clusterid_to_dataids;
    };



    
    //------------------------------------------------------------------
    // 実装部
    //------------------------------------------------------------------

    GaussianMixtureModel::GaussianMixtureModel()
    {
    }

    
    const std::vector< std::vector<std::size_t> >& GaussianMixtureModel::getClusters() const
    {
        return m_clusterid_to_dataids;
    }


    const std::vector<std::size_t>& GaussianMixtureModel::getCluster(const std::size_t cluster_id) const
    {
        return m_clusterid_to_dataids.at(cluster_id);
    }
    

    template<class DataType>
    bool GaussianMixtureModel::clustering(const std::size_t dim, const std::vector<DataType> &dataset_stl, const std::size_t num_clusters, std::vector< std::vector<double> > &centroids)
    {
        // set parameter
        const std::size_t N(dataset_stl.size());
        m_dim = dim;
        m_num_clusters = num_clusters;
        Eigen::MatrixXd dataset( scl::toEigenMatrix(dim, dataset_stl) );
        

        // initialize
        //initialize(dataset);
        initialize2(dataset_stl);


        // reset gamma
        m_gamma.clear();
        m_gamma.resize(N, std::vector<double>(num_clusters, 0.0));


        // EM algorithm
        bool is_converged(false);
        double pre_bic(-100);
        for (std::size_t itr = 0; itr < 100; ++itr)
        {
            // calc gamma
            expectationStep(dataset);

            // calc mean, pi, covariance
            maximizationStep(dataset);

            // calc BIC
            double bic = calcBIC(dataset);
            if ( (bic - pre_bic) > 0.001 )
            {
                pre_bic = bic;
            }
            else
            {
                is_converged = true;
                break;
            }
        }


        // labelling
        m_clusterid_to_dataids.clear();
        m_clusterid_to_dataids.resize(num_clusters);
        for (std::size_t j = 0; j < N; ++j)
        {
            std::size_t best_cluster_id(0);
            double best_score(0.0);
            for ( std::size_t k = 0; k < num_clusters; ++k)
            {
                double score = m_pi[k] * scl::normal::probabilityDensityFunction(dataset.row(j), m_mean[k], m_covariance[k]);
                if (score > best_score)
                {
                    best_cluster_id = k;
                    best_score = score;
                }
            }
            m_clusterid_to_dataids[best_cluster_id].push_back(j);
        }


        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < num_clusters; ++j)
            {
                printf("%8.4f", m_gamma[i][j]);
            }
            std::cout << std::endl;
        }
        std::cout << "------------------------" << std::endl;
        for (int i = 0; i < num_clusters; ++i)
        {
            printf("%8.4f", m_pi[i]);
        }
        std::cout << std::endl;
        std::cout << pre_bic << std::endl;
    }

    
    template<class DataType>
    void GaussianMixtureModel::initialize2(const std::vector<DataType> &dataset)
    {
        // set parameter
        const std::size_t dim(m_dim);
        const std::size_t num_clusters(m_num_clusters);
        const std::size_t N(dataset.size());
        

        // k-means initialize
        scl::KMeans kmeans;
        std::vector< std::vector<double> > centroids;
        kmeans.clustering(dim, dataset, num_clusters, centroids);
        m_clusterid_to_dataids = kmeans.getClusters();


        // reset data
        m_mean.resize(num_clusters, Eigen::VectorXd::Zero(dim));
        m_pi.resize(num_clusters, 0.0);
        m_covariance.resize(num_clusters, Eigen::MatrixXd::Zero(dim,dim));
                
        
        // init
        for (std::size_t k = 0; k < num_clusters; ++k)
        {
            double Nk(m_clusterid_to_dataids[k].size());
            
            // calc mean
            m_mean[k] = scl::toEigenVector(dim, centroids[k]);
            
            // calc pi
            m_pi[k] = Nk / static_cast<double>(N);

            // calc covariance
            // for (std::size_t j = 0; j < m_clusterid_to_dataids[k].size(); ++j)
            // {
            //     std::size_t data_index = m_clusterid_to_dataids[k][j];
            //     Eigen::VectorXd err = scl::toEigenVector(dim, dataset.at(data_index)) - m_mean[k];
            //     m_covariance[k] += err * err.transpose();
            // }
            // m_covariance[k] /= Nk;
            m_covariance[k] = Eigen::MatrixXd::Identity(dim, dim);
        }
    }
        
    
    void GaussianMixtureModel::initialize(const Eigen::MatrixXd &dataset)
    {
        // set parameter
        const std::size_t dim(m_dim);
        const std::size_t num_clusters(m_num_clusters);
        const std::size_t N(dataset.rows());

        
        // データセットのインデックスリストを作成 //
        std::vector<std::size_t> shuffle_indices(N);
        std::iota(shuffle_indices.begin(), shuffle_indices.end(), 0);

        
        // シャッフル //
        std::random_device seed_gen;
        std::mt19937 engine(seed_gen());
        std::shuffle(shuffle_indices.begin(), shuffle_indices.end(), engine);

        
        // init label
        m_clusterid_to_dataids.clear();
        m_clusterid_to_dataids.resize(num_clusters);
        for (std::size_t shuffle_index = 0; shuffle_index < shuffle_indices.size(); ++shuffle_index)
        {
            std::size_t data_index = shuffle_indices.at(shuffle_index);
            std::size_t cluster_index = data_index % num_clusters;
            m_clusterid_to_dataids[cluster_index].push_back(data_index);
        }

        
        // reset data
        m_mean.resize(num_clusters, Eigen::VectorXd::Zero(dim));
        m_pi.resize(num_clusters, 0.0);
        m_covariance.resize(num_clusters, Eigen::MatrixXd::Zero(dim,dim));
        
        
        // init
        for (std::size_t k = 0; k < num_clusters; ++k)
        {
            double Nk(m_clusterid_to_dataids[k].size());
            
            // calc mean
            for (std::size_t j = 0; j < m_clusterid_to_dataids[k].size(); ++j)
            {
                std::size_t data_index = m_clusterid_to_dataids[k][j];
                m_mean[k] += dataset.row(data_index).transpose();
            }
            m_mean[k] /= Nk;
            
            // calc pi
            m_pi[k] = Nk / static_cast<double>(N);

            // calc covariance
            for (std::size_t j = 0; j < m_clusterid_to_dataids[k].size(); ++j)
            {
                std::size_t data_index = m_clusterid_to_dataids[k][j];
                Eigen::VectorXd err = dataset.row(data_index).transpose() - m_mean[k];
                m_covariance[k] += err * err.transpose();
            }
            m_covariance[k] /= Nk;
        }
    }


    void GaussianMixtureModel::expectationStep(const Eigen::MatrixXd &dataset)
    {
        // set parameter
        const std::size_t num_clusters(m_num_clusters);


        // for each data
        for (std::size_t j = 0; j < dataset.rows(); ++j)
        {
            double dominator(0.0);
            for (std::size_t k = 0; k < num_clusters; ++k)
            {
                double pi_pdf = m_pi[k] * scl::normal::probabilityDensityFunction(dataset.row(j), m_mean[k], m_covariance[k]);
                m_gamma[j][k] = pi_pdf;
                dominator += pi_pdf;
            }
            for (std::size_t k = 0; k < num_clusters; ++k)
            {
                m_gamma[j][k] /= dominator;
            }
        }
    }

    
    void GaussianMixtureModel::maximizationStep(const Eigen::MatrixXd &dataset)
    {
        // set parameter
        const std::size_t dim(m_dim);
        const std::size_t num_clusters(m_num_clusters);
        const std::size_t N(dataset.rows());
        

        // for each cluster
        for (std::size_t k = 0; k < num_clusters; ++k)
        {
            // calc Nk
            double Nk(0.0);
            for (std::size_t j = 0; j < N; ++j)
            {
                Nk += m_gamma[j][k];
            }
            
            
            // calc mean
            m_mean[k] = Eigen::VectorXd::Zero(dim);
            for (std::size_t j = 0; j < N; ++j)
            {
                m_mean[k] += m_gamma[j][k] * dataset.row(j).transpose();
            }
            m_mean[k] /= Nk;

            
            // calc pi
            m_pi[k] = Nk / static_cast<double>(N);

            
            // calc covariance
            m_covariance[k] = Eigen::MatrixXd::Zero(dim, dim);
            for (std::size_t j = 0; j < N; ++j)
            {
                Eigen::VectorXd err = dataset.row(j).transpose() - m_mean[k];
                m_covariance[k] += m_gamma[j][k] * err * err.transpose();
            }
            m_covariance[k] /= Nk;
        }
    }


    double GaussianMixtureModel::calcBIC(const Eigen::MatrixXd &dataset)
    {
        // set parameter
        const std::size_t dim(m_dim);
        const std::size_t num_clusters(m_num_clusters);
        const std::size_t N(dataset.rows());


        double log_likelihood(0.0);
        for (std::size_t j = 0; j < N; ++j)
        {
            double pdf(0.0);
            for (std::size_t k = 0; k < num_clusters; ++k)
            {
                pdf += ( m_pi[k] * scl::normal::probabilityDensityFunction(dataset.row(j), m_mean[k], m_covariance[k]) );
            }
            log_likelihood += std::log(pdf);
        }

        
        double p(m_dim);
        double q = p * (p + 1) * 0.5 * num_clusters;
        q += ( p * num_clusters );
        q += ( static_cast<double>(num_clusters) - 1.0);
        double bic = -2.0 * log_likelihood + q * std::log( static_cast<double>(N) );
        return bic;
    }
    
    
}  // end of namespace scl


#endif /* SCL_GAUSSIAN_MIXTURE_MODEL_HPP */
