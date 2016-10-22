// -*- c++ -*-

#ifndef CREEK_STEP_SEQUENCE_H
#define CREEK_STEP_SEQUENCE_H

#include "../util/matrixTypes.hpp"
#include "../interpolator/RotationInterpolator.h"
#include <deque>

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
      rfoot.translation() = creek::Vector3::Zero();
      rfoot.linear() = creek::Matrix3::Identity();

      lfoot.translation() = creek::Vector3::Zero();
      lfoot.linear() = creek::Matrix3::Identity();

      sup = DFOOT;
    }
    creek::Position rfoot, lfoot;
    creek::FootType sup;
  };


  //-----------------------------------------------------------------------------------------------------

  
  class OneStepSequence
  {
  public:
    OneStepSequence(double in_dt);
    //~OneStepSequence();

    void init(const creek::Position &in_rfoot, const creek::Position &in_lfoot);
    void set(double in_time);  // only double support phase
    void set(const creek::Position &in_swing_foot, creek::FootType in_swing_foot_type, double in_single_time, double in_double_time);

    void get(creek::StepData &out_step);

    inline bool empty() { return m_remain_count <= 0; }
    
    inline double stepHeight() const { return m_step_height; }
    inline double& stepHeight() { return m_step_height; }
    
    inline creek::FootType supportType() const { return m_sup; }


  private:
    void calcCycloidStep(const creek::Vector3 &start, const creek::Vector3 &goal);
    //void calcQuinticStep();

    double m_dt;
    int m_remain_count;
    int n1, n2;

    double m_step_height;
    creek::Position m_start_rfoot, m_start_lfoot;
    creek::Position m_goal_rfoot, m_goal_lfoot;
    creek::FootType m_sup;

    std::deque<creek::Vector3> m_seq_pos;
    creek::RotationInterpolator m_seq_rot;
  };


  //-----------------------------------------------------------------------------------------------------


  // class StepSequence
  // {
  // public:
  //   StepSequence();
  //   ~StepSequence();
  // };
}

#endif
