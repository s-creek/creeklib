#ifndef CREEK_MATRIX_TYPES_HPP
#define CREEK_MATRIX_TYPES_HPP


//------------------------------------------------------------------
#ifdef USE_CNOID_MODEL

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace creek {
  using Eigen::Matrix2d;
  using Eigen::Vector2d;
  using Eigen::Matrix3d;
  using Eigen::Vector3d;
  using Eigen::Vector4d;
  using Eigen::MatrixXd;
  using Eigen::VectorXd;
  using Eigen::Quaterniond;
  using Eigen::AngleAxisd;

  typedef Eigen::Matrix3d Matrix2;
  typedef Eigen::Vector3d Vector2;
  typedef Eigen::Matrix3d Matrix3;
  typedef Eigen::Vector3d Vector3;
  typedef Eigen::Vector4d Vector4;

  typedef Eigen::MatrixXd dmatrix;
  typedef Eigen::VectorXd dvector;

  typedef Eigen::Quaterniond Quaternion;
  typedef Eigen::AngleAxisd  AngleAxis;

  typedef Eigen::Transform<double, 3, Eigen::AffineCompact> Position;

  class dzeromatrix : public dmatrix
  {
  public:
    dzeromatrix(std::size_t row, std::size_t col) : dmatrix(dmatrix::Zero(row, col)) {}
  };

  class didentity : public dmatrix
  {
  public:
    didentity(std::size_t row, std::size_t col) : dmatrix(dmatrix::Identity(row, col)) {}
  };
  
  class dzerovector : public dvector
  {
  public:
    dzerovector(std::size_t n) : dvector(dvector::Zero(n)) {}
  };
  
  class dunit : public dvector
  {
  public:
    dunit(std::size_t size, std::size_t index) : dvector(dvector::Unit(size, index)) {}
  };
  
}


//------------------------------------------------------------------
#elif defined USE_HRP_MODEL

#include "extendedBoostMatrix.hpp"

#include "tvmetForwardDeclaration.hpp" 
#include "extendedTvmetVector.hpp"
#include "extendedTvmetMatrix.hpp"
#include "tvmetQuaternion.hpp"
#include "tvmetAngleAxis.hpp"

namespace creek {
  typedef creek_tvmet::Matrix<double,2,2> Matrix2;
  typedef creek_tvmet::Vector<double,2>   Vector2;
  typedef creek_tvmet::Matrix<double,3,3> Matrix3;
  typedef creek_tvmet::Vector<double,3>   Vector3;
  typedef creek_tvmet::Vector<double,4>   Vector4;
  typedef creek_tvmet::Quaternion<double> Quaternion;
  typedef creek_tvmet::AngleAxis<double>  AngleAxis;


  struct Position
  {
  public:
    Matrix3 linear() const { return m_mat; }
    Matrix3& linear() { return m_mat; }
    
    Vector3 translation() const { return m_vec; }
    Vector3& translation() { return m_vec; }
    
    void setIdentity() {
      m_vec = Vector3::Zero();
      m_mat = Matrix3::Identity();
    }

  private:
    Matrix3 m_mat;
    Vector3 m_vec;
  };
}

template<class matT, class valT>
tvmet::CommaInitializer<matT, matT::Size> operator<<(matT& mat, valT rhs) {
  return tvmet::CommaInitializer<matT, matT::Size>(mat, rhs);
}

#endif

#endif // CREEK_MATRIX_TYPES_HPP
