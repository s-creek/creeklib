// -*- c++ -*-

#ifndef CREEK_ROTATION_INTERPOLATOR_H
#define CREEK_ROTATION_INTERPOLATOR_H

#include "Interpolator.h"
#include "../util/matrixTypes.hpp"

namespace creek
{
  class RotationInterpolator
  {
  public:
    RotationInterpolator(double in_dt);
    ~RotationInterpolator();

    void init(const creek::Matrix3& in_sR);
    
    void set(const creek::Matrix3& in_gR, double time, InterpolationType in_itype=CUBIC, double in_delta=0.0);
    bool calc();


  private:
    Interpolator m_inter;

    // start
    creek::Matrix3 m_sR;

    // goals
    struct TargetR
    {
      creek::Matrix3 gR;
      double time, delta;
    };
    std::deque<TargetR> m_goals;
  };
}

#endif
