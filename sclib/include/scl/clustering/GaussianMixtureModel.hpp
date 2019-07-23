/**
 * @file GaussianMixtureModel.hpp
 */

#ifndef SCL_GAUSSIAN_MIXTURE_MODEL_HPP
#define SCL_GAUSSIAN_MIXTURE_MODEL_HPP

#include <scl/util/Statistics.hpp>
#include <scl/util/EigenUtil.hpp>
#include <vector>

namespace scl
{
    /**
     * @class GaussianMixtureModel
     * @brief Gaussian Mixture Model (GMM : 混合ガウスモデル)
     * @details クラスタ数 = 正規分布数 @n
     * 参考サイト @n
     * <a href="https://datachemeng.com/gaussianmixturemodel/">GMM</a> @n
     * <a href="http://nonbiri-tereka.hatenablog.com/entry/2014/07/29/100733">code</a> 
     */
    class GaussianMixtureModel
    {
    public:
        /* for eigen */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** @brief コンストラクタ */
        GaussianMixtureModel();

        
    private:
        /** @brief クラスタリングするデータの次数 */
        std::size_t m_dim;

        /** @brief 負担率 (N:データ数) x (k:クラスター数) */
        std::vector< std::vector<double> > m_gamma;

        /** @brief 各クラスタの正規化時の平均 (k x dim) */
        std::vector< Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > m_mean;

        /** @brief 各クラスタの分散共分散 (k x dim x dim) */
        std::vector< Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > m_covariance;

        /** @brief 各クラスタの混合係数 (各正規分布の重み) */
        std::vector<double> m_pi;
    };



    
    //------------------------------------------------------------------
    // 実装部
    //------------------------------------------------------------------

    GaussianMixtureModel::GaussianMixtureModel()
    {
    }
    
    
}  // end of namespace scl


#endif /* SCL_GAUSSIAN_MIXTURE_MODEL_HPP */
