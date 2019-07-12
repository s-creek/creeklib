#include "StepSequence.h"
#include "discretization.hpp"
#include <iostream>  // debug

using namespace creek;

StepSequence::StepSequence(double in_dt)
  : m_dt(in_dt),
    m_seq_rot(in_dt)
{
  m_remain_count = 0;
  m_sup = DFOOT;
  m_stepType = WAIT;
  m_step_height = 0.05;
  m_step_time = 0.0;
}


void StepSequence::init(const Position &in_rfoot, const Position &in_lfoot)
{
  m_start_rfoot = in_rfoot;
  m_start_lfoot = in_lfoot;

  m_goal_rfoot = m_start_rfoot;
  m_goal_lfoot = m_start_lfoot;
}


void StepSequence::set(double in_time, StepType in_stepType)
{
  m_goal_rfoot = m_start_rfoot;
  m_goal_lfoot = m_start_lfoot;
  m_sup = DFOOT;
  m_stepType = in_stepType;

  // time to step number (discretization)
  m_remain_count = timeToNum(in_time, m_dt);
  n1 = n2 = (m_remain_count + 1);

  m_step_time = m_remain_count*m_dt;

  m_seq_pos.clear();
  m_seq_rot.clear();
}


void StepSequence::set(const creek::StepData &in_step)
{
  // time to step number (discretization)
  n2 = timeToNum(in_step.double_time/2.0, m_dt);
  n1 = n2 + timeToNum(in_step.single_time, m_dt);
  m_remain_count = n1 + n2;

  m_step_time = m_remain_count*m_dt;


  // set support foot type (for next step)
  m_sup = in_step.sup;


  // set step type
  m_stepType = in_step.stepType;


  // set start & goal foot data
  Position start_swing_foot, goal_swing_foot;
  switch(m_sup)
    {
    case RFOOT:
      {
	// set swing foot
	start_swing_foot = m_start_lfoot;
	goal_swing_foot  = in_step.lfoot;

	// set goal
	m_goal_rfoot = m_start_rfoot;
	m_goal_lfoot = in_step.lfoot;
      }
      break;

    case LFOOT:
      {
	// set swing foot
	start_swing_foot = m_start_rfoot;
	goal_swing_foot  = in_step.rfoot;

	// set goal
	m_goal_rfoot = in_step.rfoot;
	m_goal_lfoot = m_start_lfoot;
      }
      break;

    default:
      set(m_step_time, in_step.stepType);
      return;
    }


  // calc swing foot trajectory
  m_seq_pos.clear();
  m_seq_rot.clear();
  switch(in_step.stepType)
    {
    case QUINTIC_STEP:
      {
	calcInterpolationStep(start_swing_foot.translation(), goal_swing_foot.translation(), in_step.single_time, QUINTIC, 0.5);
	m_seq_rot.calc(start_swing_foot.linear(), goal_swing_foot.linear(), in_step.single_time, TWO_AXIS, QUINTIC);
      }
      break;

    case CUBIC_STEP:
      {
	calcInterpolationStep(start_swing_foot.translation(), goal_swing_foot.translation(), in_step.single_time, CUBIC, 0.5);
	m_seq_rot.calc(start_swing_foot.linear(), goal_swing_foot.linear(), in_step.single_time, TWO_AXIS, CUBIC);
      }
      break;

    case CYCLOID_STEP:
      {
	calcCycloidStep(start_swing_foot.translation(), goal_swing_foot.translation());
	m_seq_rot.calc(start_swing_foot.linear(), goal_swing_foot.linear(), in_step.single_time);
      }
      break;

    case STEP_STOP:
    case STEP_START:
    default:
      set(m_step_time, in_step.stepType);
    }
}


void StepSequence::wait(double in_time)
{
  int wait_count = timeToNum(in_time, m_dt);
  m_remain_count += wait_count;
  n1 += wait_count;
  n2 += wait_count;

  m_step_time += wait_count*m_dt;
}


void StepSequence::get(StepData &out_step, bool pop)
{
  if( m_remain_count > 0 ) {
    if( pop )
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
      Vector3 pos( m_seq_pos.front() );  if( m_seq_pos.size() > 1 && pop ) m_seq_pos.pop_front();
      Matrix3 rot;  m_seq_rot.get(rot, pop);

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
	}
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


    // set step type
    out_step.stepType = m_stepType;


    // end sequence
    if( m_remain_count == 0 ) {
      m_start_rfoot = m_goal_rfoot;
      m_start_lfoot = m_goal_lfoot;

      m_seq_pos.clear();
      m_seq_rot.clear();
    }
  }
  else {
    out_step.rfoot = m_start_rfoot;
    out_step.lfoot = m_start_lfoot;
    out_step.sup   = DFOOT;
    out_step.stepType = m_stepType;
  }
}


void StepSequence::calcCycloidStep(const creek::Vector3 &in_start, const creek::Vector3 &in_goal)
{
  int n = n1 - n2;
  double dtheta = 2 * M_PI / n;
  Vector3 dr(in_goal - in_start);


  for(int i=1; i<n; ++i) {
    double theta = i*dtheta;
    double rate = (theta - std::sin(theta)) / (2*M_PI);

    double dz = m_step_height * 0.5 * (1 - std::cos(theta));
    Vector3 tmp(in_start + rate*dr);
    tmp(2) += dz;
    m_seq_pos.push_back(tmp);
  }
  m_seq_pos.push_back( Vector3(in_goal) );
}


void StepSequence::calcInterpolationStep(const creek::Vector3 &in_start, const creek::Vector3 &in_goal, double in_time, InterpolationType in_itype, double in_top_ratio)
{
  // start --z1--> top --z2 --> goal
  double time_z1, time_z2;
  time_z1 = in_time * in_top_ratio;
  time_z2 = in_time - time_z1;

  double dz = in_goal(2) - in_start(2);


  // set start & gola
  double start_xy[] = { in_start(0), in_start(1) };
  double goal_xy[]  = { in_goal(0), in_goal(1) };

  double start_z = in_start(2);
  double mid_z   = m_step_height + dz * in_top_ratio;
  double goal_z  = in_goal(2);


  // calc interpolation
  Interpolator seq_xy(2, m_dt), seq_z(1, m_dt);
  seq_xy.calc(&start_xy[0], &goal_xy[0], in_time, in_itype);
  seq_z.init(&start_z);
  seq_z.set(&mid_z, time_z1, in_itype);
  seq_z.set(&goal_z, time_z2, in_itype);
 

  // add sequence
  while( !seq_xy.empty() || !seq_z.empty() ) {
    double xy[2], z;
    seq_xy.get(&xy[0]);
    seq_z.get(&z);
 
    Vector3 pos;
    pos << xy[0], xy[1], z;
    m_seq_pos.push_back(pos);
  }
}


double StepSequence::groundHeight(creek::FootType foot_type)
{
  StepData step;
  get(step, false);

  double ground_height=0;
  switch(foot_type)
    {
    case RFOOT:
      ground_height = step.rfoot.translation()(2);
      break;
    case LFOOT:
      ground_height = step.lfoot.translation()(2);
      break;
    case DFOOT:
      ground_height = (step.rfoot.translation()(2) + step.lfoot.translation()(2)) / 2.0;
      break;
    default:
      std::cerr << "in the air" << std::endl;
    }

  return ground_height;
}
