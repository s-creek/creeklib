#ifndef CREEK_EXTENDED_BOOST_MATRIX_HPP
#define CREEK_EXTENDED_BOOST_MATRIX_HPP

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace creek
{
  namespace ublas = boost::numeric::ublas;
    
  //typedef ublas::matrix<double, ublas::column_major> dmatrix;
  typedef ublas::zero_matrix<double> dzeromatrix;
  typedef ublas::identity_matrix<double> didentity;

  typedef ublas::vector<double> dvector;
  typedef ublas::zero_vector<double> dzerovector;
  typedef ublas::unit_vector<double> dunit;

  
  class dmatrix : public ublas::matrix<double, ublas::column_major>
  {
  private:
    typedef ublas::matrix<double, ublas::column_major> base;

  public:
    //
    // constructor
    //
    dmatrix() : base() {}
    dmatrix(std::size_t row, std::size_t col) : base(row, col) {}
    dmatrix(const base &m) : base(m) {}
    

    //
    // public function
    //
    static inline dmatrix Identity(std::size_t row, std::size_t col) {
      dmatrix ret(ublas::identity_matrix<double>(row,col));
      return ret;
    }

    static inline dmatrix Zero(std::size_t row, std::size_t col) {
      dmatrix ret(ublas::zero_matrix<double>(row,col));
      return ret;
    }
    

    //
    // for operator
    //
    dmatrix& operator = (const dmatrix &m) {
      base::operator=(m);
      return *this;
    }

    template<class C> 
    dmatrix& operator = (const ublas::matrix_container<C> &m) {
      base::operator=(m);
      return *this;
    }

    template<class AE>
    dmatrix& operator = (const ublas::matrix_expression<AE> &ae) {
      base::operator=(ae);
      return *this;
    }
    
    dmatrix operator * (const dmatrix &m) {
      dmatrix ret(ublas::prod(*this, m));
      return ret;
    }
    
  };
  
}

#endif
