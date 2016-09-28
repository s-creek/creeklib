#ifndef CREEK_MATRIX_TYPES_HPP
#define CREEK_MATRIX_TYPES_HPP


//------------------------------------------------------------------
#ifdef USE_CNOID_MODEL

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace creek {
  using Eigen::Matrix3d;
  using Eigen::Vector3d;
  using Eigen::MatrixXd;
  using Eigen::VectorXd;

  typedef Eigen::Matrix3d Matrix3;
  typedef Eigen::Vector3d Vector3;

  typedef Eigen::MatrixXd dmatrix;
  typedef Eigen::VectorXd dvector;

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

#include "extendedTvmetVector.hpp"
#include "extendedTvmetMatrix.hpp"
#include "extendedBoostMatrix.hpp"

namespace creek {
  typedef creek::Matrix<double,3,3> Matrix3;
  typedef creek::Vector<double,3>   Vector3;
}

template<class matT, class valT>
tvmet::CommaInitializer<matT, matT::Size> operator<<(matT& mat, valT rhs) {
  return tvmet::CommaInitializer<matT, matT::Size>(mat, rhs);
}

#endif

#endif // CREEK_MATRIX_TYPES_HPP