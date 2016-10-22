#include "StepSequence.h"
#include <iostream>  // debug

using namespace creek;


int timeToNum(double time, double dt)
{
  return ( time + 0.5*dt ) / dt;
}


OneStepSequence::OneStepSequence(double in_dt)
  : m_dt(in_dt),
    m_seq_rot(in_dt)
{
  m_remain_count = 0;
  m_sup = DFOOT;
  m_step_height = 0.05;
}


void OneStepSequence::init(const Position &in_rfoot, const Position &in_lfoot)
{
  m_start_rfoot = in_rfoot;
  m_start_lfoot = in_lfoot;
}


void OneStepSequence::set(double in_time)
{
  m_goal_rfoot = m_start_rfoot;
  m_goal_lfoot = m_start_lfoot;


  // time to step number (discretization)
  m_remain_count = timeToNum(in_time, m_dt);
  n1 = n2 = (m_remain_count + 1);


  m_seq_pos.clear();
  m_seq_rot.clear();
}


void OneStepSequence::set(const Position &in_swing_foot, FootType in_swing_foot_type, double in_single_time, double in_double_time)
{
  // time to step number (discretization)
  n2 = timeToNum(in_double_time/2.0, m_dt);
  n1 = n2 + timeToNum(in_single_time, m_dt);
  m_remain_count = n1 + n2;


  // set support foot type (for next step)
  m_sup = reverseFoot(in_swing_foot_type);


  // calc swing foot
  switch(in_swing_foot_type)
    {
    case RFOOT:
      {
	// set goal
	m_goal_rfoot = in_swing_foot;
	m_goal_lfoot = m_start_lfoot;

	// calc position
	calcCycloidStep(m_start_rfoot.translation(), m_goal_rfoot.translation());

	// calc rotation
	m_seq_rot.init(m_start_rfoot.linear());
	m_seq_rot.set(m_goal_rfoot.linear(), in_single_time);
      }
      break;

    case LFOOT:
      {
	// set goal
	m_goal_rfoot = m_start_rfoot;
	m_goal_lfoot = in_swing_foot;

	// calc position
	calcCycloidStep(m_start_lfoot.translation(), m_goal_lfoot.translation());

	// calc rotation
	m_seq_rot.init(m_start_lfoot.linear());
	m_seq_rot.set(m_goal_lfoot.linear(), in_single_time);
      }
      break;

    default:
      std::cerr << "can not move foot" << std::endl;
    };
}


void OneStepSequence::get(StepData &out_step)
{
  if( m_remain_count > 0 ) {
    --m_remain_count;

    //
    // double support phase
    //
    if( m_remain_count >= n1 ) {
      out_step.rfoot = m_start_rfoot;
      out_step.lfoot = m_start_lfoot;
      out_step.sup   = DFOOT;
    }
    //
    // single support phase
    //
    else if( m_remain_count >= n2 ) {
      Vector3 pos( m_seq_pos.front() );  m_seq_pos.pop_front();
      Matrix3 rot;  m_seq_rot.get(rot);

      switch(m_sup)
	{
	case RFOOT:
	  out_step.rfoot = m_goal_rfoot;
	  out_step.lfoot.translation() = pos;
	  out_step.lfoot.linear() = rot;
	  break;

	case LFOOT:
	  out_step.rfoot.translation() = pos;
	  out_step.rfoot.linear() = rot;
	  out_step.lfoot = m_goal_lfoot;
	  break;

	default:
	  std::cerr << "can not move foot" << std::endl;
	};
      out_step.sup = m_sup;
    }
    //
    // double support phase
    //
    else {
      out_step.rfoot = m_goal_rfoot;
      out_step.lfoot = m_goal_lfoot;
      out_step.sup   = DFOOT;
    }

    // end sequence
    if( m_remain_count == 0 ) {
      m_start_rfoot = m_goal_rfoot;
      m_start_lfoot = m_goal_lfoot;
    }
  }
  else {
    out_step.rfoot = m_start_rfoot;
    out_step.lfoot = m_start_lfoot;
    out_step.sup   = DFOOT;
  }
}


void OneStepSequence::calcCycloidStep(const creek::Vector3 &start, const creek::Vector3 &goal)
{
  int n = n1 - n2;
  double dtheta = 2 * M_PI / n;
  Vector3 dr(goal - start);


  m_seq_pos.clear();
  for(int i=1; i<n; ++i) {
    double theta = i*dtheta;
    double rate = (theta - std::sin(theta)) / (2*M_PI);

    double dz = m_step_height * 0.5 * (1 - std::cos(theta));
    Vector3 tmp(start + rate*dr);
    tmp(2) += dz;
    m_seq_pos.push_back(tmp);
  }
  m_seq_pos.push_back( Vector3(goal) );
}
