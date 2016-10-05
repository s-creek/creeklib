#ifndef CREEK_TVMET_ANGLE_AXIS_HPP
#define CREEK_TVMET_ANGLE_AXIS_HPP

namespace creek_tvmet
{
  template<class T>
  class AngleAxis
  {
  public:
    typedef T  Scalar;
    typedef Vector< T, 3 >    Vector3;
    typedef Matrix< T, 3, 3 > Matrix3;
    typedef Quaternion<T>     QuaternionType;
    

  protected:
    Vector3 m_axis;
    Scalar  m_angle;


  public:
    inline AngleAxis() {}

    template<class Derived>
    inline AngleAxis(const Scalar& angle, const Vector<Derived, 3>& axis)
    {
      m_angle = angle;
      for(unsigned int i=0; i<3; i++)  m_axis(i) = axis(i); 
    }
    template<class Derived>
    inline AngleAxis(const Quaternion<Derived>& q)
    {
      *this = q;
    }
    template<class Derived>
    inline AngleAxis(const Matrix<Derived,3,3>& mat)
    {
      *this = mat;
    }
    template<class Derived>
    inline AngleAxis(const AngleAxis<Derived>& other)
    {
      *this = other;
    }


    //
    // function
    //
    inline Scalar angle() const { return m_angle; }
    inline Vector3 axis() const { return m_axis; }

    inline Scalar& angle() { return m_angle; }
    inline Vector3& axis() { return m_axis; }

    template<typename Derived>
    AngleAxis& fromRotationMatrix(const Matrix<Derived,3,3>& mat);

    Matrix3 toRotationMatrix() const;

    inline AngleAxis inverse() const { return AngleAxis(-m_angle, m_axis); }


    //
    // operator
    //
    inline QuaternionType operator* (const AngleAxis& other) const
    { 
      return QuaternionType(*this) * QuaternionType(other);
    }
    inline QuaternionType operator* (const QuaternionType& other) const
    {
      return QuaternionType(*this) * other;
    }
    template<class Derived>
    AngleAxis& operator=(const Quaternion<Derived>& q);

    template<class Derived>
    AngleAxis& operator=(const Matrix<Derived,3,3>& mat);

    template<class Derived>
    AngleAxis& operator=(const AngleAxis<Derived>& other);
  };


  //---------------------------------------------------------------------------------


  template<typename Scalar>
  template<typename Derived>
  AngleAxis<Scalar>& AngleAxis<Scalar>::fromRotationMatrix(const Matrix<Derived,3,3>& mat)
  {
    return *this = QuaternionType(mat);
  }


  template<typename Scalar>
  typename AngleAxis<Scalar>::Matrix3
  AngleAxis<Scalar>::toRotationMatrix() const
  {
    Matrix3 ret;

    const double sth = std::sin(m_angle);
    const double vth = 1.0 - std::cos(m_angle);
  
    double ax = m_axis(0);
    double ay = m_axis(1);
    double az = m_axis(2);
  
    const double axx = ax*ax*vth;
    const double ayy = ay*ay*vth;
    const double azz = az*az*vth;
    const double axy = ax*ay*vth;
    const double ayz = ay*az*vth;
    const double azx = az*ax*vth;

    ax *= sth;
    ay *= sth;
    az *= sth;

    ret = 1.0 - azz - ayy, -az + axy,       ay + azx,
      az + axy,        1.0 - azz - axx, -ax + ayz,
      -ay + azx,       ax + ayz,        1.0 - ayy - axx;

    return ret;
  }


  template<class Scalar>
  template<class Derived>
  inline AngleAxis<Scalar>& AngleAxis<Scalar>::operator=(const Quaternion<Derived>& q)
  {
    Scalar n = q.vec().norm();
    if( n < std::numeric_limits<Scalar>::epsilon() )
      n = Scalar(0);

    if (n != Scalar(0))
      {
	m_angle = Scalar(2)*std::atan2(n, std::abs(q.w()));
	if(q.w() < 0)
	  n = -n;
	m_axis  = q.vec() / n;
      }
    else
      {
	m_angle = Scalar(0);
	m_axis << Scalar(1), Scalar(0), Scalar(0);
      }
    return *this;
  }


  template<class Scalar>
  template<class Derived>
  inline AngleAxis<Scalar>& AngleAxis<Scalar>::operator=(const Matrix<Derived,3,3>& mat)
  {
    return *this = QuaternionType(mat);
  }


  template<class Scalar>
  template<class Derived>
  inline AngleAxis<Scalar>& AngleAxis<Scalar>::operator=(const AngleAxis<Derived>& other)
  {
    m_angle = other.angle();
    for(unsigned int i=0; i<3; i++)
      m_axis(i) = other.axis()(i);

    return *this;
  }
}

#endif
