// -*- c++ -*-

#ifndef CREEK_CAPTURE_POINT_H
#define CREEK_CAPTURE_POINT_H

#include "../util/matrixTypes.hpp"
#include <deque>

namespace creek
{
  class CapturePoint
  {
  public:
#ifdef USE_CNOID_MODEL
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

    CapturePoint(double dt);

    // !!! cp(2) = ground height !!!
    void init(const creek::Vector3 &com, const creek::Vector3 &cp);
    
    void set(const creek::Vector3 &cp, double time, double com_height);
    inline void set(const creek::Vector3 &cp, double time) { set(cp, time, m_com_height); }
    bool stop(double time);

    void get(creek::Vector3 &zmp, creek::Vector3 &com, creek::Vector3 &cp, bool pop=true);
    inline void getGoal(creek::Vector3 &zmp, creek::Vector3 &com, creek::Vector3 &cp) {
      zmp = m_zmp;
      com = m_goal_com;
      cp  = m_goal_cp;
    }
    inline void getStart(creek::Vector3 &zmp, creek::Vector3 &com, creek::Vector3 &cp) {
      zmp = m_zmp;
      com = m_start_com;
      cp  = m_start_cp;
    }


    creek::Vector3 expectedZmp(const creek::Vector3 &start_cp, const creek::Vector3 &goal_cp, double time, double com_height);
    inline creek::Vector3 expectedZmp(const creek::Vector3 &goal_cp, double time, double com_height) { return expectedZmp(m_start_cp, goal_cp, time, com_height); }
    inline creek::Vector3 expectedZmp(const creek::Vector3 &goal_cp, double time) { return expectedZmp(m_start_cp, goal_cp, time, m_com_height); }
    inline creek::Vector3 expectedZmp(const creek::Vector3 &start_cp, const creek::Vector3 &goal_cp, double time) { return expectedZmp(start_cp, goal_cp, time, m_com_height); }


    // goal_foot : next support foot position
    // offset_direction : default y direction offset (world)
    creek::Vector3 calcNextCapturePoint(const creek::Vector3 &goal_foot, const creek::Vector3 &offset_direction, const creek::Vector3 &velocity, double time, double com_height);
    inline creek::Vector3 calcNextCapturePoint(const creek::Vector3 &goal_foot, const creek::Vector3 &offset_direction,  const creek::Vector3 &velocity, double time) {
      return calcNextCapturePoint(goal_foot, offset_direction, velocity, time, m_com_height);
    }


    // calc default y direction offset
    void calcDefaultOffset(double distance, double time, double com_height);  // before init
    void calcOffset(double distance, double time, double com_height);         // after init
    inline void calcOffset(double distance, double time) { calcOffset(distance, time, m_com_height); }

    inline void setDefaultOffset(double offset) { m_offset = offset; }
    inline double offset() { return m_offset; }


    inline bool empty() { return m_seq_com.empty() || m_seq_cp.empty(); }
    inline int numSequence() { return m_seq_com.size(); } 
    inline double remainingTime() { return m_seq_com.size()*m_dt; }
    inline double comHeight() { return m_com_height; } 
    inline const creek::Vector3& zmp() const { return m_zmp; }


  private:
    double m_dt;
    double m_com_height;
    double m_offset;

    creek::Vector3 m_zmp;
    creek::Vector3 m_start_com, m_start_cp;
    creek::Vector3 m_goal_com, m_goal_cp;
#ifdef USE_CNOID_MODEL
    std::deque<creek::Vector3, Eigen::aligned_allocator<creek::Vector3> > m_seq_com, m_seq_cp;
#else
    std::deque<creek::Vector3> m_seq_com, m_seq_cp;
#endif
  };
}

#endif
