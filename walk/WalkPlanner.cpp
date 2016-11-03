#include "WalkPlanner.h"
#include "CapturePoint.h"
#include "BipedRobot.h"
#include "Geometry.h"

#include <iostream>
#include <cstdio>

using namespace creek;

WalkPlanner::WalkPlanner(double in_dt)
  : m_stepping(false),
    m_dt(in_dt)
{
  m_stepSequence = new StepSequence(in_dt);
  m_capturePoint = new CapturePoint(in_dt);
}


void WalkPlanner::setOffset(double in_offset)
{
  m_capturePoint->setDefaultOffset(in_offset);
}


void WalkPlanner::calcOffset(double in_distance, double in_time, double in_com_height)
{
  m_capturePoint->calcDefaultOffset(in_distance, in_time, in_com_height);
}


FootType WalkPlanner::getSupportFootType(const StepData &step, const Vector3 &in_zmp, double in_margin)
{
  FootType ret(AIR);

  Vector3  com   = step.com;
  Position rfoot = step.rfoot;
  Position lfoot = step.lfoot;

  Vector3 relate(in_zmp-com);
  double  dz = relate(2);  // for DOUBLE
  if( std::fabs(dz) < 1.0e-12 )
    return ret;


  // RIGHT
  if( ret == AIR ) {
    Vector3 n(rfoot.linear()*Vector3::UnitZ());
    Vector4 plane;
    getPlaneEquation(rfoot.translation(), n, plane);
    
    Vector3 rzmp;
    if( getIntersectPlaneAndLine(plane, com, in_zmp, rzmp) ) {
      
      Vector3 distR;
      distR = rfoot.linear().transpose() * (rzmp-rfoot.translation());
      if( distR(0) < (m_footSize(0)-in_margin)      // toe
	  && distR(0) > -(m_footSize(1)-in_margin)  // heel
	  && distR(1) > -(m_footSize(2)-in_margin)  // outer
	  && distR(1) < m_footSize(3))              // inner
	ret = RFOOT;
    }
  }

  // LEFT
  if( ret == AIR ) {
    Vector3 n(lfoot.linear()*Vector3::UnitZ());
    Vector4 plane;
    getPlaneEquation(lfoot.translation(), n, plane);
    
    Vector3 lzmp;
    if( getIntersectPlaneAndLine(plane, com, in_zmp, lzmp) ) {

      Vector3 distL;
      distL = lfoot.linear().transpose() * (lzmp-lfoot.translation());
      if( distL(0) < (m_footSize(0)-in_margin)      // toe
	  && distL(0) > -(m_footSize(1)-in_margin)  // heel
	  && distL(1) < (m_footSize(2)-in_margin)   // outer
	  && distL(1) > -m_footSize(3))             // inner
	ret = LFOOT;
    }
  }

  // DOUBLE
  if( ret == AIR ) {
    Vector3 rt, rh, lt, lh;  // RorL, TOEorHEEL
    rt = rfoot.translation() + rfoot.linear() * Vector3(m_footSize(0)-in_margin, m_footSize(3), 0.0) - com;
    rh = rfoot.translation() + rfoot.linear() * Vector3(-m_footSize(1)+in_margin, m_footSize(3), 0.0) - com;
    lt = lfoot.translation() + lfoot.linear() * Vector3(m_footSize(0)-in_margin, -m_footSize(3), 0.0) - com;
    lh = lfoot.translation() + lfoot.linear() * Vector3(-m_footSize(1)+in_margin, -m_footSize(3), 0.0) - com;

    // projection
    rt = rt / rt[2] * dz + com;
    rh = rh / rh[2] * dz + com;
    lt = lt / lt[2] * dz + com;
    lh = lh / lh[2] * dz + com;

    Vector3 rt2lt(lt-rt), rt2rh(rh-rt), lh2lt(lt-lh), lh2rh(rh-lh);
    Vector3 rt2zmp(in_zmp-rt), lh2zmp(in_zmp-lh);
    if( rt2lt.cross(rt2zmp)[2] >= 0.0 
	&& rt2rh.cross(rt2zmp)[2] <= 0.0 
	&& lh2rh.cross(lh2zmp)[2] >= 0.0
	&& lh2lt.cross(lh2zmp)[2] <= 0.0)
      ret = DFOOT;
  }
  return ret;
}


creek::FootType WalkPlanner::getSwingFootType(const creek::Position &rfootRef, const creek::Position &lfootRef)
{
  creek::FootType ret(creek::DFOOT);

  StepData step;
  getGoal(step);

  bool mr = needLiftUpFoot(step.rfoot, rfootRef);
  bool ml = needLiftUpFoot(step.lfoot, lfootRef);
  if( mr && ml ) {
    ret = creek::DFOOT;
  }
  else if( mr ) {
    ret = creek::RFOOT;
  }
  else if( ml ) {
    ret = creek::LFOOT;
  }
  else {
    ret = creek::AIR;
  }
  return ret;
}


void WalkPlanner::init(creek::BipedRobotPtr in_robot)
{
  // init step sequence
  m_stepSequence->init(in_robot->rfoot()->position(), in_robot->lfoot()->position());


  // init capture point
  Vector3 com(in_robot->body()->calcCenterOfMass());
  Vector3 cp(com);
  cp(2) = m_stepSequence->groundHeight(DFOOT);
  m_capturePoint->init(com, cp);
}


bool WalkPlanner::empty()
{
  return (m_stepSequence->empty() || m_capturePoint->empty()) && m_goals.empty();
}


double WalkPlanner::remainingTime()
{
  double time = m_stepSequence->remainingTime();
  for(unsigned int i=0; i<m_goals.size(); ++i)
    time += (m_goals.at(i).single_time+m_goals.at(i).double_time);
  
  return time;
}


void WalkPlanner::get(creek::StepData &step, bool pop)
{
  m_stepSequence->get(step, pop);
  m_capturePoint->get(step.zmp, step.com, step.cp, pop);
  step.com_height = m_capturePoint->comHeight();

  if( m_stepSequence->empty() || m_capturePoint->empty() )
    calcStep();
}


void WalkPlanner::getGoal(creek::StepData &step)
{
  if( m_goals.empty() ) {
    m_stepSequence->getGoal(step);
    m_capturePoint->getGoal(step.zmp, step.com, step.cp);
    step.com_height = step.com(2) - step.zmp(2);
  }
  else {
    step = m_goals.back();
  }
}


void WalkPlanner::getStart(creek::StepData &step)
{
  m_stepSequence->getStart(step);
  m_capturePoint->getStart(step.zmp, step.com, step.cp);
  step.com_height = m_capturePoint->comHeight();
}


void WalkPlanner::addStep(const creek::Position &in_swing_foot, creek::FootType in_swing_foot_type, creek::StepType in_step_type)
{
  StepData step;
  getGoal(step);

  switch(in_swing_foot_type)
    {
    case RFOOT:
      step.rfoot = in_swing_foot;
      step.sup = LFOOT;
      break;

    case LFOOT:
      step.lfoot = in_swing_foot;
      step.sup = RFOOT;
      break;

    case DFOOT:
    default:
      step.sup = DFOOT;
      break;
    }
  step.stepType = in_step_type;

  step.single_time = m_single_time;
  step.double_time = m_double_time;

  addStep(step);
}


void WalkPlanner::addStep(const creek::StepData &in_step)
{
  StepData step = in_step;
  double time = step.single_time+step.double_time;
  if( time < 0.1 ) {
    step.single_time = m_single_time;
    step.double_time = m_double_time;
  }

  m_goals.push_back(step);
  if( m_stepSequence->empty() || m_capturePoint->empty() )
    calcStep();
}


bool WalkPlanner::start()
{
  bool need_start(false);

  if( m_goals.empty() )
    return true;


  //
  // calc reference capture point & zmp
  //
  StepData start, goal;
  getStart(start);
  goal = m_goals.front();

  double time = goal.single_time+goal.double_time;
  Vector3 goal_pos, offset_direction, distance, velocity;
  switch(goal.sup)
    {
    case RFOOT:
      {
	goal_pos = goal.lfoot.translation();
	offset_direction = -goal.lfoot.linear() * Vector3::UnitY();
	distance = goal_pos - start.lfoot.translation();
	velocity = distance / time / 2.0;  // 足に大して体幹の速度は半分なので / 2.0
	goal.cp  = m_capturePoint->calcNextCapturePoint(goal_pos, offset_direction, velocity, time, goal.com_height);
      }
      break;

    case LFOOT:
      {
	goal_pos = goal.rfoot.translation();
	offset_direction = goal.rfoot.linear() * Vector3::UnitY();
	distance = goal_pos - start.rfoot.translation();
	velocity = distance / time / 2.0;  // 足に大して体幹の速度は半分なので / 2.0
	goal.cp  = m_capturePoint->calcNextCapturePoint(goal_pos, offset_direction, velocity, time, goal.com_height);
      }
      break;

    case DFOOT:
    default:
      {
	// goal_pos = 0.5*(goal.rfoot.translation()+goal.lfoot.translation());
	// offset_direction = Vector3::Zero();
	// distance = Vector3::Zero();
	// velocity = Vector3::Zero();
      }
    }
  goal.zmp = m_capturePoint->expectedZmp(goal.cp, time, goal.com_height);
  m_goals.front().cp = goal.cp;
  

  //
  //  reference zmp in support foot ?
  //
  FootType expected_support_type = getSupportFootType(start, goal.zmp, m_margin);
  if( (goal.sup == DFOOT) && (expected_support_type != AIR) )
    expected_support_type = DFOOT;
  
  if( goal.sup != expected_support_type ) {
    need_start = true;

    double offset_scale(1.0);
    switch(goal.sup)
      {
      case RFOOT:
	goal_pos = start.rfoot.translation();
	offset_direction = offset_scale * start.rfoot.linear() * Vector3::UnitY();
	break;

      case LFOOT:
	goal_pos = start.lfoot.translation();
	offset_direction = -offset_scale * start.lfoot.linear() * Vector3::UnitY();
	break;
	
      case DFOOT:
      default:
	goal_pos = 0.5*(start.rfoot.translation()+start.lfoot.translation());
	offset_direction = Vector3::Zero();
      }
    start.cp  = m_capturePoint->calcNextCapturePoint(goal_pos, offset_direction, Vector3::Zero(), time, start.com_height);
    start.zmp = m_capturePoint->expectedZmp(start.cp, time, start.com_height);
  
    m_stepSequence->set(time, STEP_START);
    m_capturePoint->set(start.cp, time, start.com_height);
  
    //
    // debug
    //
    {
      Vector3 gcom, gcp, gzmp;
      m_capturePoint->getGoal(gzmp, gcom, gcp);
      std::cerr << "WalkPlanner : start" << std::endl;
      std::cerr << "  expected sup type " << expected_support_type << ",  input sup type = " << goal.sup << std::endl;
      std::printf("  ref zmp = [ %7.4f,  %7.4f,  %7.4f ]\n", gzmp(0), gzmp(1), gzmp(2));
      std::printf("  ref com = [ %7.4f,  %7.4f,  %7.4f ]\n", gcom(0), gcom(1), gcom(2));
      std::printf("  ref cp  = [ %7.4f,  %7.4f,  %7.4f ]\n", gcp(0), gcp(1), gcp(2));
    }
  }

  return need_start;
}


void WalkPlanner::stop(bool immediately)
{
  if( immediately )
    m_goals.clear();


  StepData step;
  getGoal(step);
  if( step.stepType == STEP_STOP || step.stepType == WAIT )
    return;


  step.cp  = 0.5*(step.rfoot.translation()+step.lfoot.translation());
  step.sup = DFOOT;
  step.stepType = WAIT;
  addStep(step);

  step.stepType = STEP_STOP;
  addStep(step);
}


bool WalkPlanner::needLiftUpFoot(const creek::Position &in_start, const creek::Position &in_goal)
{
  // if( m_stepping )
  //   return true;

  double dpmin(1.0e-6), dRmin(1.0e-6);
  bool need(false);
 
  Vector3 dp(in_goal.translation() - in_start.translation());
  Matrix3 dR(in_goal.linear().transpose() * in_start.linear());
  creek::AngleAxisd omega(dR);
  if( dp.norm() > dpmin || std::fabs(omega.angle()) > dRmin )
    need = true; 

  return need;
}


void WalkPlanner::calcStep()
{
  if( m_goals.empty() )
    return;

  // start to walk
  if( start() )
    return;

  switch( m_goals.front().stepType )
    {
    case CYCLOID_STEP:
    case QUINTIC_STEP:
    case CUBIC_STEP:
      {
	addStepPlane();
      }
      break;

    case STEP_STOP:
    case STEP_START:
    case WAIT:
    default:
      {
	addComMove();
      }
    }
  m_goals.pop_front();
}


void WalkPlanner::addStepPlane()
{
  StepData goal = m_goals.front();
  double time = goal.single_time+goal.double_time;
  m_stepSequence->set(goal);
  m_capturePoint->set(goal.cp, time, goal.com_height);
}


void WalkPlanner::addStepStaiars()
{
  // todo
}


void WalkPlanner::addComMove()
{
  StepData goal = m_goals.front();
  double time = goal.single_time+goal.double_time;
  
  if( goal.stepType == STEP_STOP ) {
    m_stepping = false;
    if( m_capturePoint->stop(time) )
      m_stepSequence->wait(time);
  }
  else {
    m_stepSequence->set(time, goal.stepType);
    m_capturePoint->set(goal.cp, time, goal.com_height);
  }
}
