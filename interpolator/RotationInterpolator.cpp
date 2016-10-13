#include "RotationInterpolator.h"

using namespace creek;

RotationInterpolator::RotationInterpolator(double in_dt)
  : m_inter(1, in_dt)
{
  m_sR = Matrix3::Identity();
  m_inter.setAutoCalc(false);
}


RotationInterpolator::~RotationInterpolator()
{
  m_inter.clear();
  m_goals.clear();
}


void RotationInterpolator::init(const Matrix3& in_sR)
{
  m_sR = in_sR;
  double sx(0);
  m_inter.init(&sx);
}


void RotationInterpolator::set(const Matrix3& in_gR, double time, InterpolationType in_itype, double in_delta)
{
  double gx(1.0);
  m_inter.set(&gx, time, in_itype, in_delta);
}


bool RotationInterpolator::calc()
{
  return true;
}
