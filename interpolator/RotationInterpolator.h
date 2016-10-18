// -*- c++ -*-

#ifndef CREEK_ROTATION_INTERPOLATOR_H
#define CREEK_ROTATION_INTERPOLATOR_H

#include "Interpolator.h"
#include "../util/matrixTypes.hpp"

namespace creek
{
  enum InterpolationTypeR {
    TWO_AXIS=0,
    SLERP
  };
}

namespace creek
{
  class RotationInterpolator
  {
  public:
    RotationInterpolator(double in_dt);
    ~RotationInterpolator();

    void init(const creek::Matrix3& in_sR);
    
    void set(const creek::Matrix3& in_gR, double time, InterpolationTypeR in_itypeR=TWO_AXIS, InterpolationType in_itype=CUBIC, double in_delta=0.0);
    bool calc();
    
    bool get(creek::Matrix3 &out_R, bool popp=true);
    inline creek::Matrix3 get(bool popp=true) {
      creek::Matrix3 out;
      get(out, popp);
      return out;
    }

    inline bool empty() {
      return m_inter.empty() && m_goals.empty();
    }

    // for TWO_AXIS
    inline void setAxis(const Vector3& in_axis) { 
      m_axis = in_axis;
      m_axis.normalize();
    }


  private:
    Interpolator m_inter;
    
    // start
    creek::Matrix3 m_sR;

    // goals
    struct TargetR
    {
      creek::Matrix3 gR;
      double time, delta;
      InterpolationType itype;
      InterpolationTypeR itypeR;
    };
    std::deque<TargetR> m_goals;
    creek::Matrix3 m_gR;


    InterpolationTypeR m_itypeR;

    // for SLERP
    Quaternion m_sQ, m_gQ;

    // for TWO_AXIS
    creek::Vector3 m_axis;
    creek::AngleAxis m_aaa, m_aab;
  };
}

#endif
