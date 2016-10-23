#include "RotationInterpolator.h"
#include <iostream>  // debug

using namespace creek;

RotationInterpolator::RotationInterpolator(double in_dt)
  : m_inter(1, in_dt)
{
  m_sR = Matrix3::Identity();
  m_gR = Matrix3::Identity();
  m_inter.setAutoCalc(false);
  m_axis << 0,0,1;
}


RotationInterpolator::~RotationInterpolator()
{
  m_inter.clear();
  m_goals.clear();
}


void RotationInterpolator::init(const Matrix3& in_sR)
{
  m_sR = in_sR;
}


void RotationInterpolator::set(const Matrix3& in_gR, double time, InterpolationTypeR in_itypeR, InterpolationType in_itype, double in_delta)
{
  TargetR target;
  target.gR     = in_gR;
  target.time   = time;
  target.itypeR = in_itypeR;
  target.itype  = in_itype;
  target.delta  = in_delta;

  m_goals.push_back(target);


  if( m_inter.seqEmpty() )
    calc();
}


bool RotationInterpolator::calc()
{
  if( m_goals.empty() )
    return false;

  TargetR target = m_goals.front();
  m_itypeR = target.itypeR;
  m_gR = target.gR;


  // variation
  double sx(0), gx(1.0);
  m_inter.calc(&sx, &gx, target.time, target.itype, target.delta);


  switch( m_itypeR )
    {
    case SLERP:
      {
	m_sQ = m_sR;
	m_gQ = target.gR;
      }
      break;

    case TWO_AXIS:
      {
	// 誤差回転行列Rの導出
	Matrix3 dR(m_sR.transpose() * target.gR);

	// target方向への回転の導出
	double cos_angle_a = m_axis.dot( Vector3(dR * m_axis) );
	if( cos_angle_a > (1.0 - 1.0e-6) )
	  m_aaa.angle() = 0;
	else if(cos_angle_a < -(1.0 - 1.0e-6) )
	  m_aaa.angle() = M_PI;
	else
	  m_aaa.angle() = std::acos(cos_angle_a);

	if( m_aaa.angle() > 1e-3 ) {
	  m_aaa.axis() = m_axis.cross( Vector3(dR * m_axis) );
	  m_aaa.axis().normalize();
	}
	else{
	  m_aaa.axis() = Vector3::UnitX();
	}
	// target方向以外への回転の導出
	Matrix3 R_a(m_aaa.toRotationMatrix());
	Matrix3 R_b(R_a.transpose() * dR);
	m_aaa.fromRotationMatrix(R_b);
      }
      break;

    default:
      std::cerr << "interpolation type (" << m_itypeR << ") not supported" << std::endl;
    }
  

  m_goals.pop_front();
  return true;
}


bool RotationInterpolator::get(creek::Matrix3 &out_R, bool popp)
{
  bool toNextTerm(false);


  // no sequence
  if( m_inter.seqEmpty() ) {
    out_R = m_gR;
    return false;
  }


  double variation;
  m_inter.get(&variation, popp);


  switch(m_itypeR)
    {
    case SLERP:
      out_R = m_sQ.slerp(variation, m_gQ).toRotationMatrix();
      break;

    case TWO_AXIS:
      {
	AngleAxis aaa(m_aaa.angle()*variation, m_aaa.axis());
	AngleAxis aab(m_aab.angle()*variation, m_aab.axis());
	out_R = m_sR * aaa.toRotationMatrix() * aab.toRotationMatrix();
      }
      break;

    default:
      std::cerr << "interpolation type (" << m_itypeR << ") not supported" << std::endl;
      out_R = Matrix3::Identity();
    }


  if( m_inter.seqEmpty() ) {
    out_R = m_gR;
    m_sR  = m_gR;
    toNextTerm = calc();
  }
  return toNextTerm;
}


double RotationInterpolator::remainingTime()
{
  double time = m_inter.remainingTimeToFirstGoal();
  for(unsigned int i=0; i<m_goals.size(); i++)
    time += m_goals.at(i).time;

  return time;
}


double RotationInterpolator::remainingTimeToFirstGoal()
{
  return m_inter.remainingTimeToFirstGoal();
}
