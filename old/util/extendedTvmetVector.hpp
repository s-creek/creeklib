#ifndef CREEK_EXTENDED_TVMET_VECTOR_HPP
#define CREEK_EXTENDED_TVMET_VECTOR_HPP

#include <tvmet/Vector.h>

namespace creek_tvmet
{
  template<class T, std::size_t Sz>
  class Vector : public tvmet::Vector< T, Sz >
  {
  private:
    typedef tvmet::Vector< T, Sz > tvector;

  public:
    //
    // public types
    //
    typedef T  value_type;
    typedef T* iterator;

    enum {
      Size = Sz		/**< The size of the vector. */
    };
    

    //
    // constructor
    //
    explicit Vector() : tvector() {}
    Vector(const tvector &rhs) : tvector(rhs) {}

    template<class InputIterator>
    explicit Vector(InputIterator first, InputIterator last) : tvector(first, last) {}

    template<class InputIterator>
    explicit Vector(InputIterator first, std::size_t sz) : tvector(first, sz) {}

    explicit Vector(value_type rhs) : tvector(rhs) {}

    template<class E>
    explicit Vector(const tvmet::XprVector<E, Sz>& e) : tvector(e) {}
    

    //
    // public function
    //
    // static inline Vector Unit() {
    //   return Vector( tvmet::identity<value_type, Cols, Rows>() );
    // }

    static inline Vector Zero() {
      return Vector(0);
    }

    inline value_type norm() const {
      return tvmet::norm2(*this);
    }

    inline value_type squaredNorm() const {
      return static_cast<typename tvmet::NumericTraits<value_type>::float_type>(tvmet::dot(*this, *this));
    }

    inline void normalize() {
      *this = Vector(tvmet::normalize(*this));
    }

    inline Vector normalized() const {
      return Vector(tvmet::normalize(*this));
    }

    inline value_type dot(const tvector& in_v) const {
      return tvmet::dot(*this, in_v);
    }

    inline Vector cross(const tvector& in_v) const {
      return Vector(tvmet::cross(*this, in_v));
    }
      
    //
    // for operator
    //
    tvmet::CommaInitializer<Vector, Size> operator=(value_type rhs) {
      return tvmet::CommaInitializer<Vector, Size>(*this, rhs);
    }

    tvmet::CommaInitializer<Vector, Size> operator<<(value_type rhs) {
      return tvmet::CommaInitializer<Vector, Size>(*this, rhs);
    }

    enum {
      ops_assign = Size,
      ops        = ops_assign,
      use_meta   = ops < TVMET_COMPLEXITY_M_ASSIGN_TRIGGER ? true : false
    };

  private:
    template<class T2, class Assign>
    void assign_to(Vector<T2, Size>& dest, const Assign& assign_fn) const {
      do_assign(tvmet::dispatch<use_meta>(), dest, *this, assign_fn);
    }
    
  public:
    template<class T2>
    Vector& operator=(const Vector<T2, Size>& rhs) {
      rhs.assign_to(*this, tvmet::Fcnl_assign<value_type, T2>());
      return *this;
    }

    /** assign a given XprVector element wise to this vector. */
    template<class E>
    Vector& operator=(const tvmet::XprVector<E, Size>& rhs) {
      rhs.assign_to(*this, tvmet::Fcnl_assign<value_type, typename E::value_type>());
      return *this;
    }


  public:
    static Vector UnitX() {
      Vector ret(0);
      if( Sz > 0 )
	ret(0) = 1;
      return ret;
    }

    static Vector UnitY() {
      Vector ret(0);
      if( Sz > 1 )
	ret(1) = 1;
      return ret;
    }

    static Vector UnitZ() {
      Vector ret(0);
      if( Sz > 2 )
	ret(2) = 1;
      return ret;
    }
  };
}


#endif
