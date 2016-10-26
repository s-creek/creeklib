#include "CapturePoint.h"
#include "../interpolator/Interpolator.h"
#include "discretization.hpp"
#include <iostream>  // debug

using namespace creek;

CapturePoint::CapturePoint(double in_dt)
  : m_dt(in_dt)
{
  m_com_height = 1.0;

  m_start_com = Vector3::Zero();
  m_start_cp  = Vector3::Zero();

  m_offset = 0.0;
}


void CapturePoint::init(const creek::Vector3 &in_com, const creek::Vector3 &in_cp)
{
  // set initial data
  m_start_com = in_com;
  m_start_cp  = in_cp;
  m_zmp << in_com(0), in_com(1), in_cp(2);

  // reset goal data
  m_goal_com = in_com;
  m_goal_cp  = in_cp;

  // reset sequence
  m_seq_com.clear();
  m_seq_cp.clear();

  // calc com height
  m_com_height = in_com(2) - in_cp(2);
}


//
// ground height = start_cp(2) while this step
//
void CapturePoint::set(const Vector3 &in_cp, double in_time, double in_com_height)
{
  m_com_height = m_start_com(2) - m_start_cp(2);
  double ground_height = m_start_cp(2);


  // calc com pos z (world)
  double start_com_z(m_start_com(2)), goal_com_z(in_cp(2)+in_com_height);
  Interpolator seq_com_z(1, m_dt);
  seq_com_z.calc(&start_com_z, &goal_com_z, in_time, CUBIC);


  // calc capture point parameter
  static double g(9.80665);
  double mid_com_height = (m_com_height + goal_com_z-ground_height)/2.0;
  double ww  = std::sqrt(g/mid_com_height);
  double bb  = std::exp(ww*in_time);
  double wdt = ww * m_dt;

  // calc zmp
  m_zmp = 1/(1-bb)*in_cp - bb/(1-bb)*m_start_cp;
  m_zmp(2) = ground_height;
  

  // calc com & cp sequence
  m_seq_com.clear();
  m_seq_cp.clear();
  Vector3 com(m_start_com), cp(m_start_cp);
  while( !seq_com_z.empty() ) {
    double com_z;
    seq_com_z.get(&com_z);
 
    cp  = m_zmp + std::exp(wdt)*(cp-m_zmp);
    com = cp + std::exp(-wdt)*(com-cp);

    cp(2)  = ground_height;
    com(2) = com_z;
    
    m_seq_com.push_back(com);
    m_seq_cp.push_back(cp);
  }

  m_goal_com   = m_seq_com.back();
  m_goal_cp    = m_seq_cp.back();
  m_com_height = in_com_height;
}


void CapturePoint::get(Vector3 &out_zmp, Vector3 &out_com, Vector3 &out_cp)
{
  out_zmp = m_zmp;

  if( empty() ) {
    out_com = m_start_com;
    out_cp  = m_start_cp;
  }
  else {
    out_com = m_seq_com.front();
    out_cp  = m_seq_cp.front();

    m_seq_com.pop_front();
    m_seq_cp.pop_front();

    // end sequence
    if( empty() ) {
      m_start_com = m_goal_com;
      m_start_cp  = m_goal_cp;
    }
  }
}


Vector3 CapturePoint::expectedZmp(const Vector3 &in_start_cp, const Vector3 &in_goal_cp, double in_time, double in_com_height)
{
  static double g(9.80665);
  double mid_com_height = (m_com_height + in_com_height)/2.0;
  double ww = std::sqrt(g/mid_com_height);
  double bb = std::exp(ww*in_time);

  Vector3 zmp;
  zmp = 1/(1-bb)*in_goal_cp - bb/(1-bb)*in_start_cp;

  return zmp;
}


Vector3 CapturePoint::calcNextCapturePoint(const Vector3 &in_goal_foot, const Vector3 &in_offset_direction, const Vector3 &in_velocity, double in_time, double in_com_height)
{
  static double g(9.80665);
  double mid_com_height = (m_com_height + in_com_height)/2.0;
  double ww = std::sqrt(g/mid_com_height);
  double bb = std::exp(ww*in_time);

  Vector3 default_offset, add_offset;
  default_offset = m_offset * in_offset_direction;
  add_offset = 1/(bb-1)*in_time*in_velocity;
  //add_offset = 2/(bb-1)*in_time*in_velocity;

  Vector3 next_cp;
  next_cp = in_goal_foot + default_offset + add_offset;
  return next_cp;
}


void CapturePoint::calcDefaultOffset(double in_distance, double in_time, double in_com_height)
{
  static double g(9.80665);
  double mid_com_height = (m_com_height + in_com_height)/2.0;
  double ww = std::sqrt(g/mid_com_height);
  double bb = std::exp(ww*in_time);

  m_offset = 1/(bb-1)*std::fabs(in_distance);
}
