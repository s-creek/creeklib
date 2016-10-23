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
    CapturePoint(double dt);

    // !!! cp(2) = ground height !!!
    void init(const creek::Vector3 &com, const creek::Vector3 &cp);
    
    void set(const creek::Vector3 &cp, double time, double com_height);
    inline void set(const creek::Vector3 &cp, double time) { set(cp, time, m_com_height); }

    void get(creek::Vector3 &zmp, creek::Vector3 &com, creek::Vector3 &cp);

    creek::Vector3 expectedZmp(const creek::Vector3 &cp, double time, double com_height);
    inline creek::Vector3 expectedZmp(const creek::Vector3 &cp, double time) { return expectedZmp(cp, time, m_com_height); }

    inline bool empty() { return m_seq_com.empty() || m_seq_cp.empty(); }

  private:
    double m_dt;
    double m_com_height;

    creek::Vector3 m_start_com, m_start_cp, m_zmp;
    std::deque<creek::Vector3> m_seq_com, m_seq_cp;
  };
}

#endif
