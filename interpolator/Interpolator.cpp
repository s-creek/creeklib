#include "Interpolator.h"
#include <cstring>   // memcpy
#include <cmath>     // pow
#include <iostream>  // debug

using namespace creek;


Interpolator::Interpolator(unsigned int in_dim, double in_dt)
  : m_dim(in_dim),
    m_dt(in_dt),
    m_memsize(in_dim * sizeof(double))
{
  m_sx = new double[in_dim];
  m_sv = new double[in_dim];
  m_sa = new double[in_dim];

  for(int i=0; i<in_dim; i++) {
    m_sx[i] = m_sv[i] = m_sa[i] = 0.0;
  }
}


Interpolator::~Interpolator()
{
  clear();
  delete [] m_sx;
  delete [] m_sv;
  delete [] m_sa;
}


void Interpolator::init(const double *in_sx, const double *in_sv, const double *in_sa)
{
  for(int i=0; i<m_dim; i++) {
    m_sx[i] = in_sx[i];
    in_sv == NULL ? m_sv[i] = 0.0 : m_sv[i] = in_sv[i];
    in_sa == NULL ? m_sa[i] = 0.0 : m_sa[i] = in_sa[i];
  }
}


void Interpolator::set(const double *in_gx, double time, InterpolationType in_itype, double in_delta)
{
  set(in_gx, NULL, NULL, time, in_itype, in_delta);
}


void Interpolator::set(const double *in_gx, const double *in_gv, double time, InterpolationType in_itype, double in_delta)
{
  set(in_gx, in_gv, NULL, time, in_itype, in_delta);
}


void Interpolator::set(const double *in_gx, const double *in_gv, const double *in_ga, double time, InterpolationType in_itype, double in_delta)
{
  Target target;
  target.gx = new double[m_dim];
  target.gv = new double[m_dim];
  target.ga = new double[m_dim];
  
  for(int i=0; i<m_dim; i++) {
    target.gx[i] = in_gx[i];
    in_gv == NULL ? target.gv[i] = 0.0 : target.gv[i] = in_gv[i];
    in_ga == NULL ? target.ga[i] = 0.0 : target.ga[i] = in_ga[i];
  }
  target.time  = time;
  target.itype = in_itype;
  target.delta = in_delta;

  m_goals.push_back(target);


  if( m_seqx.empty() )
    calc();
}


void Interpolator::calc()
{
  if( m_goals.empty() )
    return;


  InterpolationType itype = m_goals.front().itype;
  switch( itype )
    {
    case LINEAR:
      linear_interpolation();
      break;

    case CUBIC:
    case QUINTIC:
    case HOFFARBIB:
      calcInterpolation(itype);
      break;
      
    case QUARTIC_LINEAR:
      quartic_linear();
      break;

    default:
      std::cerr << "now coding" << std::endl;
    }


  popGoal();
}


void Interpolator::get(double* outx, bool ppop)
{
  get(outx, NULL, NULL, ppop);
}


void Interpolator::get(double* outx, double *outv, bool ppop)
{
  get(outx, outv, NULL, ppop);
}


void Interpolator::get(double* outx, double *outv, double *outa, bool ppop)
{
  if( !m_seqx.empty() ) {
    double *&xx = m_seqx.front();
    double *&vv = m_seqv.front();
    double *&aa = m_seqa.front();

    std::memcpy(m_sx, xx, m_memsize);
    std::memcpy(m_sv, vv, m_memsize);
    std::memcpy(m_sa, aa, m_memsize);

    if( ppop )
      pop();

    if( m_seqx.empty() )
      calc();
  }

  std::memcpy(outx, m_sx, m_memsize);
  if( outv != NULL ) std::memcpy(outv, m_sv, m_memsize);
  if( outa != NULL ) std::memcpy(outa, m_sa, m_memsize);
}


void Interpolator::clear()
{
  clearFirstTerm();

  while( !m_goals.empty() ) {
    popGoal();
  }
}


void Interpolator::pop()
{
  if( m_seqx.empty() )
    return;

  double *&xx = m_seqx.front();
  double *&vv = m_seqv.front();
  double *&aa = m_seqa.front();

  delete [] xx;
  delete [] vv;
  delete [] aa;

  m_seqx.pop_front();
  m_seqv.pop_front();
  m_seqa.pop_front();
}


void Interpolator::popGoal()
{
  double *&xx = m_goals.front().gx;
  double *&vv = m_goals.front().gv;
  double *&aa = m_goals.front().ga;

  delete [] xx;
  delete [] vv;
  delete [] aa;

  m_goals.pop_front();
}


void Interpolator::clearFirstTerm()
{
  while( !m_seqx.empty() ) {
    pop();
  }
}


double Interpolator::remainingTime()
{
  double time = m_dt * m_seqx.size();
  for(unsigned int i=0; i<m_goals.size(); i++)
    time += m_goals.at(i).time;

  return time;
}


double Interpolator::remainingTimeToFirstGoal()
{
  return m_dt * m_seqx.size();
}


//-----------------------------------------------------------------------------------------


void Interpolator::linear_interpolation()
{
  Target target = m_goals.front();


  double time = target.time;
  int num = ( time + 0.5*m_dt ) / m_dt;
  time = num * m_dt;
  

  double *tmpv = new double[m_dim];
  for(int i=0; i<m_dim; i++)
    tmpv[i] = ( target.gx[i] - m_sx[i] ) / time;
  

  for(int i=1; i<=num; i++) {
    double *xx = new double[m_dim];
    double *vv = new double[m_dim];
    double *aa = new double[m_dim];

    for(int j=0; j<m_dim; j++) {
      xx[j] = m_sx[j] + tmpv[j] * ( i * m_dt );
      vv[j] = tmpv[j];
      aa[j] = 0.0;
    }

    m_seqx.push_back(xx);
    m_seqv.push_back(vv);
    m_seqa.push_back(aa);
  }
}


void Interpolator::calcInterpolation(InterpolationType in_itype)
{
  Target target = m_goals.front();
  
  
  double time = target.time;
  int num = ( time + 0.5*m_dt ) / m_dt;
  time = num * m_dt;


  // calc interpolation parameter
  double a0[m_dim], a1[m_dim], a2[m_dim], a3[m_dim], a4[m_dim], a5[m_dim];
  for(int i=0; i<m_dim; i++) {
    switch(in_itype)
      {
      case CUBIC:
	a0[i] = m_sx[i];
	a1[i] = m_sv[i];
	a2[i] = ( 3*(target.gx[i]-m_sx[i])/time - 2*m_sv[i] - target.gv[i] )/time;
	a3[i] = ( 2*(m_sx[i]-target.gx[i])/time + m_sv[i] + target.gv[i] )/time/time;
	a4[i] = a5[i] = 0.0;
	break;

      case QUINTIC:
	a0[i] = m_sx[i];
	a1[i] = m_sv[i];
	a2[i] = m_sa[i]/2.0;
	a3[i] = (20*target.gx[i] - 20*m_sx[i] - ( 8*target.gv[i] + 12*m_sv[i])*time - (3*m_sa[i] -   target.ga[i])*pow(time,2)) / (2*pow(time,3));
	a4[i] = (30*m_sx[i] - 30*target.gx[i] + (14*target.gv[i] + 16*m_sv[i])*time + (3*m_sa[i] - 2*target.ga[i])*pow(time,2)) / (2*pow(time,4));
	a5[i] = (12*target.gx[i] - 12*m_sx[i] - ( 6*target.gv[i] +  6*m_sv[i])*time - (  m_sa[i] -   target.ga[i])*pow(time,2)) / (2*pow(time,5));
	break;
	
      case HOFFARBIB:
	// 実は上のQUINTCとまったく同じ
	double A,B,C;
	A = (target.gx[i]-(m_sx[i]+m_sv[i]*time+(m_sa[i]/2.0)*time*time))/pow(time,3);
	B = (target.gv[i]-(m_sv[i]+m_sa[i]*time))/(time*time);
	C = (target.ga[i]-m_sa[i])/time;

	a0[i]=m_sx[i];
	a1[i]=m_sv[i];
	a2[i]=m_sa[i]/2.0;
	a3[i]=10*A-4*B+0.5*C;
	a4[i]=(-15*A+7*B-C)/time;
	a5[i]=(6*A-3*B+0.5*C)/(time*time);
	break;

      default:
	std::cerr << "interpolation type (" << in_itype << ") not supported" << std::endl;
	a0[i] = a1[i] = a2[i] = a3[i] = a4[i] = a5[i] = 0.0;
      }
  }


  // calc interpolation
  for(int i=1; i<=num; i++) {
    double *xx = new double[m_dim];
    double *vv = new double[m_dim];
    double *aa = new double[m_dim];

    double tt = i*m_dt;
    for(int j=0; j<m_dim; j++) {
      xx[j] = a0[j] + ( a1[j] + (a2[j] + (a3[j] + (a4[j] + a5[j]*tt )*tt )*tt )*tt )*tt;
      vv[j] = a1[j] + ( 2*a2[j] + (3*a3[j] + (4*a4[j] + 5*a5[j]*tt )*tt )*tt )*tt;
      aa[j] = 2*a2[j] + ( 6*a3[j] + ( 12*a4[j] + 20*a5[j]*tt )*tt )*tt;
    }

    m_seqx.push_back(xx);
    m_seqv.push_back(vv);
    m_seqa.push_back(aa);
  }
}


void Interpolator::quartic_linear()
{
  Target target = m_goals.front();
  
  
  // start -> n1 -> n2 -> n3(time)
  double time = target.time;
  int n3 = ( time + 0.5*m_dt ) / m_dt;
  time = n3 * m_dt;

  int n1 = ( 2 * target.delta + 0.5*m_dt ) / m_dt;
  int n2 = n3 - n1;
  if( n1 >= n2 ) {
    std::cerr << "QUARTIC_LINEAR : time param error (time = " << time << ", delta = " << target.delta << ")" << std::endl;
    return;
  }


  double mm[m_dim];
  double delta = n1 * m_dt * 0.5;
  for(unsigned int i = 0; i<m_dim; i++)
    mm[i] = (target.gx[i] - m_sx[i] - delta*(m_sv[i] + target.gv[i])) / (time - 2*delta);


  int count = 1;
  double tt = 0.0;

  // quartic term
  for(; count<n1; count++) {
    double *xx = new double[m_dim];
    double *vv = new double[m_dim];
    double *aa = new double[m_dim];

    tt = count*m_dt;
    for(int i=0; i<m_dim; i++) {
      xx[i] = m_sx[i] + m_sv[i]*tt + (mm[i] - m_sv[i]) / (16.0*pow(delta,3)) * pow(tt,3) * (4*delta - tt);
      vv[i] = m_sv[i] + (mm[i] - m_sv[i]) / (16.0*pow(delta,3)) * pow(tt,2) * (12*delta - 4*tt);
      aa[i] = (mm[i] - m_sv[i]) / (16.0*pow(delta,3)) * tt * (24*delta - 12*tt);
    }

    m_seqx.push_back(xx);
    m_seqv.push_back(vv);
    m_seqa.push_back(aa);
  }

  // linear term
  for(; count<n2; count++) {
    double *xx = new double[m_dim];
    double *vv = new double[m_dim];
    double *aa = new double[m_dim];

    tt = count*m_dt;
    for(int i=0; i<m_dim; i++) {
      xx[i] = m_sx[i] + m_sv[i]*delta + mm[i]*(tt-delta);
      vv[i] = mm[i];
      aa[i] = 0.0;
    }

    m_seqx.push_back(xx);
    m_seqv.push_back(vv);
    m_seqa.push_back(aa);
  }

  // quartic term
  for(; count<n3; count++) {
    double *xx = new double[m_dim];
    double *vv = new double[m_dim];
    double *aa = new double[m_dim];

    tt = count*m_dt;
    for(int i=0; i<m_dim; i++) {
      xx[i] = target.gx[i] - target.gv[i]*(time-tt) - (mm[i] - target.gv[i]) / (16.0*pow(delta,3))*pow((time-tt),3)*(4*delta-time+tt);
      vv[i] = target.gv[i] + (mm[i] - target.gv[i]) / (16.0*pow(delta,3)) * pow((time-tt),2) * (12*delta - 4*time + 4*tt);
      aa[i] = (mm[i] - target.gv[i]) / (16.0*pow(delta,3)) * (time-tt) * ( -24*delta + 12*time - 12*tt);
    }

    m_seqx.push_back(xx);
    m_seqv.push_back(vv);
    m_seqa.push_back(aa);
  }
}
