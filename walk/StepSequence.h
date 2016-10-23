// -*- c++ -*-

#ifndef CREEK_STEP_SEQUENCE_H
#define CREEK_STEP_SEQUENCE_H

#include "../util/matrixTypes.hpp"
#include "../interpolator/RotationInterpolator.h"
#include <deque>

namespace creek
{
  enum StepType {CYCLOID_STEP, QUINTIC_STEP, CUBIC_STEP};

  enum FootType {RFOOT=-1, DFOOT, LFOOT, AIR};
  inline FootType reverseFoot(FootType type) {
    FootType ret(DFOOT);
    if( type == RFOOT )
      ret = LFOOT;
    else if( type == LFOOT )
      ret = RFOOT;
    return ret;
  }

  struct StepData
  {
    StepData()
    {
      rfoot.setIdentity();
      lfoot.setIdentity();
      sup = DFOOT;
    }
    creek::Position rfoot, lfoot;
    creek::FootType sup;
  };


  //-----------------------------------------------------------------------------------------------------

  
  class OneStepSequence
  {
  public:
    OneStepSequence(double dt);
    //~OneStepSequence();

    void init(const creek::Position &rfoot, const creek::Position &lfoot);
    inline void init(const creek::StepData &foot) {
      init(foot.rfoot, foot.lfoot);
    }

    void set(double time);  // only double support phase
    void set(const creek::StepData &step, double single_time, double double_time, creek::StepType step_type=creek::CYCLOID_STEP);

    void get(creek::StepData &out_step);

    inline bool empty() { return m_remain_count <= 0; }
    
    inline double stepHeight() const { return m_step_height; }
    inline double& stepHeight() { return m_step_height; }
    
    inline creek::FootType supportType() const { return m_sup; }

    inline double remainingTime() { return m_remain_count*m_dt; }


  private:
    void calcCycloidStep(const creek::Vector3 &start, const creek::Vector3 &goal);
    void calcInterpolationStep(const creek::Vector3 &start, const creek::Vector3 &goal, double time, creek::InterpolationType itype, double top_ration=0.5);

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
