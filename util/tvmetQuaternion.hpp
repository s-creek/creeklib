#ifndef CREEK_TVMET_QUATERNION_HPP
#define CREEK_TVMET_QUATERNION_HPP

namespace creek_tvmet
{
  template<class T>
  class Quaternion
  {
  public:
    typedef T  Scalar;
    typedef Vector< T, 4 >    Coefficients;
    typedef Vector< T, 3 >    Vector3;
    typedef Matrix< T, 3, 3 > Matrix3;
    typedef AngleAxis<T>      AngleAxisType;


  protected:
    Coefficients m_coeffs;


  public:
    inline Quaternion() {}
    inline Quaternion(Scalar w, Scalar x, Scalar y, Scalar z)
    {
      m_coeffs << x, y, z, w;
    }
    inline Quaternion(const Coefficients& in_coeffs)
    {
      m_coeffs = in_coeffs;
    }
    template<class Derived>
    inline Quaternion(const Matrix<Derived,3,3>& mat)
    {
      *this = mat;
    }
    template<class Derived>
    inline Quaternion(const Quaternion<Derived>& other)
    {
      //m_coeffs = other.coeffs();
      *this = other;
    }
    template<class Derived>
    inline Quaternion(const AngleAxis<Derived>& aa)
    {
      *this = aa;
    }


    //
    // function
    //
    inline Scalar x() const { return m_coeffs(0); }
    inline Scalar y() const { return m_coeffs(1); }
    inline Scalar z() const { return m_coeffs(2); }
    inline Scalar w() const { return m_coeffs(3); }

    inline Scalar& x() { return m_coeffs(0); }
    inline Scalar& y() { return m_coeffs(1); }
    inline Scalar& z() { return m_coeffs(2); }
    inline Scalar& w() { return m_coeffs(3); }

    inline Vector3 vec() const
    {
      Vector3 ret;
      ret << m_coeffs(0), m_coeffs(1), m_coeffs(2);
      return ret;
    }

    static inline Quaternion Identity() { return Quaternion(1, 0, 0, 0); }
    inline Quaternion& setIdentity() { m_coeffs << 0, 0, 0, 1; return *this; }

    inline Scalar squaredNorm() const { return m_coeffs.squaredNorm(); }
    inline Scalar norm() const { return m_coeffs.norm(); }

    inline const Coefficients& coeffs() const { return m_coeffs; }
    inline Coefficients& coeffs() { return m_coeffs; }

    inline void normalize() { m_coeffs.normalize(); }
    inline Quaternion normalized() const { return Quaternion(Coefficients(m_coeffs.normalized())); }

    Matrix3 toRotationMatrix() const;

    template<class Derived>
    Quaternion slerp(Scalar t, const Quaternion<Derived>& other) const;

    Quaternion inverse() const;
    Quaternion conjugate() const;


    //
    // operator
    //
    template<class Derived>
    inline Quaternion operator* (const Quaternion<Derived>& q) const;

    template<class Derived>
    inline Quaternion& operator*= (const Quaternion<Derived>& q);

    template<class Derived>
    Quaternion& operator=(const Quaternion<Derived>& other);

    template<class Derived>
    Quaternion& operator=(const Matrix<Derived,3,3>& m);

    template<class Derived>
    Quaternion& operator=(const AngleAxis<Derived>& aa);
  };


  //---------------------------------------------------------------------------------


  template<class Scalar>
  inline class Quaternion<Scalar>::Matrix3
  Quaternion<Scalar>::toRotationMatrix() const
  {
    Matrix3 res;

    const Scalar tx  = Scalar(2)*this->x();
    const Scalar ty  = Scalar(2)*this->y();
    const Scalar tz  = Scalar(2)*this->z();
    const Scalar twx = tx*this->w();
    const Scalar twy = ty*this->w();
    const Scalar twz = tz*this->w();
    const Scalar txx = tx*this->x();
    const Scalar txy = ty*this->x();
    const Scalar txz = tz*this->x();
    const Scalar tyy = ty*this->y();
    const Scalar tyz = tz*this->y();
    const Scalar tzz = tz*this->z();

    res(0,0) = Scalar(1)-(tyy+tzz);
    res(0,1) = txy-twz;
    res(0,2) = txz+twy;
    res(1,0) = txy+twz;
    res(1,1) = Scalar(1)-(txx+tzz);
    res(1,2) = tyz-twx;
    res(2,0) = txz-twy;
    res(2,1) = tyz+twx;
    res(2,2) = Scalar(1)-(txx+tyy);

    return res;
  }


  template <class Scalar>
  template<class Derived>
  Quaternion<Scalar> Quaternion<Scalar>::slerp(Scalar t, const Quaternion<Derived>& other) const
  {
    static const Scalar one = Scalar(1) - std::numeric_limits<Scalar>::epsilon();
    Scalar d(0);
    for(int i=0; i<4; i++) {
      d += this->coeffs()(i) * other.coeffs()(i);
    }
    Scalar absD = std::fabs(d);

    Scalar scale0;
    Scalar scale1;

    if (absD>=one)
      {
	scale0 = Scalar(1) - t;
	scale1 = t;
      }
    else
      {
	// theta is the angle between the 2 quaternions
	Scalar theta = std::acos(absD);
	Scalar sinTheta = std::sin(theta);

	scale0 = std::sin( ( Scalar(1) - t ) * theta) / sinTheta;
	scale1 = std::sin( ( t * theta) ) / sinTheta;
	if (d<0)
	  scale1 = -scale1;
      }

    Coefficients out_coeffs;
    out_coeffs = scale0 * coeffs() + scale1 * other.coeffs();
    return Quaternion<Scalar>(out_coeffs);
  }


  template <class Scalar>
  inline Quaternion<Scalar> Quaternion<Scalar>::inverse() const
  {
    Scalar n2 = this->squaredNorm();
    if (n2 > 0)
      {
	Coefficients tmp(conjugate().coeffs() / n2);
	return Quaternion(tmp);
      }
    else
      {
	return Quaternion(Coefficients::Zero());
      }
  }

  
  template <class Scalar>
  inline Quaternion<Scalar> Quaternion<Scalar>::conjugate() const
  {
    return Quaternion(this->w(),-this->x(),-this->y(),-this->z());
  }


  template<class Scalar, class Derived>
  inline Quaternion<Scalar> ei_quaternion_product(const Quaternion<Scalar>& a, const Quaternion<Derived>& b)
  {
    return Quaternion<Scalar>
      (
       a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
       a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
       a.w() * b.y() + a.y() * b.w() + a.z() * b.x() - a.x() * b.z(),
       a.w() * b.z() + a.z() * b.w() + a.x() * b.y() - a.y() * b.x()
       );
  }


  template <class Scalar>
  template<class Derived>
  inline Quaternion<Scalar> Quaternion<Scalar>::operator* (const Quaternion<Derived>& other) const
  {
    return ei_quaternion_product(*this,other);
  }


  template <class Scalar>
  template<class Derived>
  inline Quaternion<Scalar>& Quaternion<Scalar>::operator*= (const Quaternion<Derived>& other)
  {
    return (*this = *this * other);
  }


  template<class Scalar>
  template<class Derived>
  inline Quaternion<Scalar>& Quaternion<Scalar>::operator=(const Quaternion<Derived>& other)
  {
    //m_coeffs = other.coeffs();
    for(unsigned int i=0; i<4; i++)
      m_coeffs(i) = other.coeffs()(i);

    return *this;
  }


  template<class Scalar>
  template<class Derived>
  inline Quaternion<Scalar>& Quaternion<Scalar>::operator=(const Matrix<Derived,3,3>& m)
  {
    // 最大成分を検索
    Scalar elem[ 4 ]; // 0:x, 1:y, 2:z, 3:w
    elem[0] =  m(0,0) - m(1,1) - m(2,2) + Scalar(1);
    elem[1] = -m(0,0) + m(1,1) - m(2,2) + Scalar(1);
    elem[2] = -m(0,0) - m(1,1) + m(2,2) + Scalar(1);
    elem[3] =  m(0,0) + m(1,1) + m(2,2) + Scalar(1);

    unsigned biggestIndex = 0;
    for ( int i = 1; i < 4; i++ ) {
      if ( elem[i] > elem[biggestIndex] )
	biggestIndex = i;
    }

    if ( elem[biggestIndex] < 0.0f ) {
      std::cerr << "Quaternion : matrix convert error" << std::endl;
      return *this; // 引数の行列に間違いあり！
    }

    // 最大要素の値を算出
    Scalar v = std::sqrt( elem[biggestIndex] ) * Scalar(0.5);
    m_coeffs(biggestIndex) = v;
    Scalar mult = Scalar(0.25) / v;

    switch ( biggestIndex ) {
    case 0: // x
      m_coeffs(1) = (m(1,0) + m(0,1)) * mult;
      m_coeffs(2) = (m(0,2) + m(2,0)) * mult;
      m_coeffs(3) = (m(2,1) - m(1,2)) * mult;
      break;
    case 1: // y
      m_coeffs(0) = (m(1,0) + m(0,1)) * mult;
      m_coeffs(2) = (m(2,1) + m(1,2)) * mult;
      m_coeffs(3) = (m(0,2) - m(2,0)) * mult;
      break;
    case 2: // z
      m_coeffs(0) = (m(0,2) + m(2,0)) * mult;
      m_coeffs(1) = (m(2,1) + m(1,2)) * mult;
      m_coeffs(3) = (m(1,0) - m(0,1)) * mult;
      break;
    case 3: // w
      m_coeffs(0) = (m(2,1) - m(1,2)) * mult;
      m_coeffs(1) = (m(0,2) - m(2,0)) * mult;
      m_coeffs(2) = (m(1,0) - m(0,1)) * mult;
      break;
    }

    return *this;
  }


  template<class Scalar>
  template<class Derived>
  inline Quaternion<Scalar>& Quaternion<Scalar>::operator=(const AngleAxis<Derived>& aa)
  {
    Scalar ha = Scalar(0.5)*aa.angle();
    Scalar sa = std::sin(ha);

    m_coeffs(3) = std::cos(ha);  // w
    for(unsigned int i=0; i<3; i++)
      m_coeffs(i) = sa * aa.axis()(i);

    return *this;
  }
}

#endif
