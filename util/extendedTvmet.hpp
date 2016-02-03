#ifndef CREEK_EXTENDED_TVMET_HPP
#define CREEK_EXTENDED_TVMET_HPP

#include <tvmet/Matrix.h>

namespace creek
{
  template<class T, std::size_t NRows, std::size_t NCols>
  class Matrix : public tvmet::Matrix< T, NRows, NCols >
  {
  private:
    typedef tvmet::Matrix< T, NRows, NCols > tmatrix;

  public:
    //
    // public types
    //
    typedef T  value_type;
    typedef T* iterator;

    enum {
      Rows = NRows,			/**< Number of rows. */
      Cols = NCols,			/**< Number of cols. */
      Size = Rows * Cols		/**< Complete Size of Matrix. */
    };
    

    //
    // constructor
    //
    explicit Matrix() : tmatrix() {}
    Matrix(const tmatrix &rhs) : tmatrix(rhs) {}

    template<class InputIterator>
    explicit Matrix(InputIterator first, InputIterator last) : tmatrix(first, last) {}

    template<class InputIterator>
    explicit Matrix(InputIterator first, std::size_t sz) : tmatrix(first, sz) {}

    explicit Matrix(value_type rhs) : tmatrix(rhs) {}

    template<class E>
    explicit Matrix(const tvmet::XprMatrix<E, Rows, Cols>& e) : tmatrix(e) {}
    

    //
    // public function
    //
    inline Matrix<value_type, Cols, Rows> transpose() {
      Matrix<value_type, Cols, Rows> ret( tvmet::trans(*this) );
      return ret;
    }

    static inline Matrix Identity() {
      return Matrix( tvmet::identity<value_type, Cols, Rows>() );
    }

    static inline Matrix Zero() {
      return Matrix(0);
    }

  
    //
    // for operator
    //
    tvmet::CommaInitializer<Matrix, Size> operator=(value_type rhs) {
      return tvmet::CommaInitializer<Matrix, Size>(*this, rhs);
    }

    tvmet::CommaInitializer<Matrix, Size> operator<<(value_type rhs) {
      return tvmet::CommaInitializer<Matrix, Size>(*this, rhs);
    }

    enum {
      ops_assign = Rows * Cols,
      ops        = ops_assign,
      use_meta   = ops < TVMET_COMPLEXITY_M_ASSIGN_TRIGGER ? true : false
    };

  private:
    template<class T2, class Assign>
    void assign_to(Matrix<T2, Rows, Cols>& dest, const Assign& assign_fn) const {
      do_assign(tvmet::dispatch<use_meta>(), dest, *this, assign_fn);
    }
    
  public:
    template<class T2>
    Matrix& operator=(const Matrix<T2, Rows, Cols>& rhs) {
      rhs.assign_to(*this, tvmet::Fcnl_assign<value_type, T2>());
      return *this;
    }

    template <class E>
    Matrix& operator=(const tvmet::XprMatrix<E, Rows, Cols>& rhs) {
      rhs.assign_to(*this, tvmet::Fcnl_assign<value_type, typename E::value_type>());
      return *this;
    }

  };
}

#endif
