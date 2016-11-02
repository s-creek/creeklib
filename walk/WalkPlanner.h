// -*- c++ -*-

#ifndef CREEK_WALK_PLANNER_H
#define CREEK_WALK_PLANNER_H

#include <deque>
#include "StepSequence.h"

#include <boost/shared_ptr.hpp>
namespace creek
{
  class CapturePoint;
  class BipedRobot;
  typedef boost::shared_ptr<BipedRobot> BipedRobotPtr;
}

namespace creek
{
  class WalkPlanner
  {
  public:
    WalkPlanner(double dt);

    inline void setTime(double single_time, double double_time)  {
      m_single_time = single_time;
      m_double_time = double_time;
    }
    inline void setFootSize(double to_toe, double to_heel, double to_outer, double to_inner) {
      m_footSize << to_toe, to_heel, to_outer, to_inner;
    }
    
    // use setOffset or calcOffset
    //   offset : y direction offset
    //   distance : distance of the left and right foot
    //   time : step time
    void setOffset(double offset);
    void calcOffset(double distance, double time, double com_height);
    
    inline void setStepping(bool flag=true) { m_stepping = flag; }
    inline bool isStepping() { return m_stepping; }

    inline void setMargin(double margin) { m_margin = margin; }
    creek::FootType getSupportFootType(const creek::StepData &step, const creek::Vector3 &zmp, double margin=0);  // zmp (on world)
    creek::FootType getSwingFootType(const creek::Position &rfootRef, const creek::Position &lfootRef);

    void init(creek::BipedRobotPtr robot);
    bool empty();
    double remainingTime();

    void get(creek::StepData &step, bool pop=true);
    void getGoal(creek::StepData &step);
    void getStart(creek::StepData &step);

    void addStep(const creek::Position &swing_foot, creek::FootType swing_foot_type, creek::StepType step_type=creek::CYCLOID_STEP);
    void addStep(const creek::StepData &step);

    bool start();
    void stop(bool immediately=false);

    creek::StepSequence* stepSequence() const { return m_stepSequence; }
    creek::CapturePoint* capturePoint() const { return m_capturePoint; }


  private:
    bool needLiftUpFoot(const creek::Position &start, const creek::Position &goal);

    void calcStep();
    void addStepPlane();
    void addStepStaiars();
    void addComMove();


    creek::StepSequence *m_stepSequence;
    creek::CapturePoint *m_capturePoint;

    bool m_stepping;
    double m_dt, m_single_time, m_double_time;
    double m_margin;  // default margin for getFootType
    creek::Vector4 m_footSize; // toe, heel, outer, inner

    std::deque<creek::StepData> m_goals;
  };
}

#endif
