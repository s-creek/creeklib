// -*- c++ -*-

#ifndef CREEK_STEP_SEQUENCE_H
#define CREEK_STEP_SEQUENCE_H

#include "../util/matrixTypes.hpp"
#include "../interpolator/RotationInterpolator.h"

namespace creek
{
  enum FootType {RFOOT=-1, DFOOT, LFOOT, AIR};
  inline FootType reverseFoot(FootType in_type) {
    FootType ret(DFOOT);
    if( in_type == RFOOT )
      ret = LFOOT;
    else if( in_type == LFOOT )
      ret = RFOOT;
    return ret;
  }

  struct StepData
  {
    StepData()
    {
      rfoot.translation() = Vector3::Zero();
      rfoot.linear() = Matrix3::Identity();

      lfoot.translation() = Vector3::Zero();
      lfoot.linear() = Matrix3::Identity();

      com = Vector3::Zero();
      zmp = Vector3::Zero();
      cp  = Vector3::Zero();

      sup = DFOOT;
    }
    Position rfoot, lfoot;
    Vector3  com, zmp, cp;
    FootType sup;
  };


  //-----------------------------------------------------------------------------------------------------

  
  class OneStepSequence
  {
  public:
    OneStepSequence();
    ~OneStepSequence();

    void init(const creek::Position &in_rfoot, const creek::Position &in_lfoot, const creek::Vector3 &in_com);
  };


  //-----------------------------------------------------------------------------------------------------


  class StepSequence
  {
  public:
    StepSequence();
    ~StepSequence();
  };
}

#endif
