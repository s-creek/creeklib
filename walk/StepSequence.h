// -*- c++ -*-

#ifndef CREEK_STEP_SEQUENCE_H
#define CREEK_STEP_SEQUENCE_H

#include "../util/matrixTypes.hpp"
#include "../interpolator/RotationInterpolator.h"
#include <deque>

namespace creek
{
  enum StepType {CYCLOID_STEP, QUINTIC_STEP, CUBIC_STEP, STEP_STOP, STEP_START, WAIT};

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
      stepType = CYCLOID_STEP;

      single_time = double_time = 0.0;
    }
    creek::Position rfoot, lfoot;
    creek::FootType sup;
    creek::StepType stepType;

    // for capture point
    creek::Vector3  com, zmp, cp;
    double com_height;

    // for walk planner
    double single_time, double_time;
  };


  //-----------------------------------------------------------------------------------------------------

  
  class StepSequence
  {
  public:
    StepSequence(double dt);
    //~StepSequence();

    void init(const creek::Position &rfoot, const creek::Position &lfoot);
    inline void init(const creek::StepData &foot) {
      init(foot.rfoot, foot.lfoot);
    }

    void set(double time, creek::StepType stepType=creek::WAIT);  // only double support phase
    void set(const creek::StepData &step);
    void wait(double time);  // add wait time after step

    void get(creek::StepData &out_step, bool pop=true);
    inline void getGoal(creek::StepData &goal) {
      goal.rfoot = m_goal_rfoot;
      goal.lfoot = m_goal_lfoot;
      goal.sup = m_sup;
      goal.stepType = m_stepType;
    }
    inline void getStart(creek::StepData &start) {
      start.rfoot = m_start_rfoot;
      start.lfoot = m_start_lfoot;
      start.sup = m_sup;
      start.stepType = m_stepType;
    }

    inline bool empty() { return m_remain_count <= 0; }
    
    inline double stepHeight() const { return m_step_height; }
    inline double& stepHeight() { return m_step_height; }
    
    inline creek::FootType supportType() const { return m_sup; }
    inline creek::StepType stepType() const { return m_stepType; }

    inline double remainingTime() { return m_remain_count*m_dt; }
    inline int numSequence() { return m_remain_count; }

    inline double stepTime() { return m_step_time; }

    double groundHeight(creek::FootType foot_type);


  private:
    void calcCycloidStep(const creek::Vector3 &start, const creek::Vector3 &goal);
    void calcInterpolationStep(const creek::Vector3 &start, const creek::Vector3 &goal, double time, creek::InterpolationType itype, double top_ration=0.5);

    double m_dt;
    int m_remain_count;
    int n1, n2;
    double m_step_time;

    double m_step_height;
    creek::Position m_start_rfoot, m_start_lfoot;
    creek::Position m_goal_rfoot, m_goal_lfoot;
    creek::FootType m_sup;
    creek::StepType m_stepType;

    std::deque<creek::Vector3>  m_seq_pos;
    creek::RotationInterpolator m_seq_rot;
  };
}

#endif
