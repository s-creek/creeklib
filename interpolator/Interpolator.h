// -*- c++ -*-

#ifndef CREEK_INTERPOLATOR_H
#define CREEK_INTERPOLATOR_H

#include <deque>

namespace creek
{
  enum InterpolationType {
    LINEAR=0,
    CUBIC,
    QUINTIC,
    QUARTIC_LINEAR,  // 4-1-4
    HOFFARBIB        // = QUINTIC
  };
}


namespace creek 
{
  class Interpolator 
  {
  public:
    Interpolator(unsigned int in_dim, double in_dt);
    ~Interpolator();

    void init(const double *in_sx, const double *in_sv=NULL, const double *in_sa=NULL);
    
    void set(const double *in_gx, double time, InterpolationType in_itype=LINEAR, double in_delta=0.0);
    void set(const double *in_gx, const double *in_gv, double time, InterpolationType in_itype=CUBIC, double in_delta=0.0);
    void set(const double *in_gx, const double *in_gv, const double *in_ga, double time, InterpolationType in_itype=QUINTIC, double in_delta=0.0);
    void calc();

    void get(double* outx, bool ppop=true);
    void get(double* outx, double *outv, bool ppop=true);
    void get(double* outx, double *outv, double *outa, bool ppop=true);

    void clear();
    void pop();
    
    double remainingTime();
    double remainingTimeToFirstGoal();

    inline bool empty() {
      return m_goals.empty() && m_seqx.empty();
    }
    inline unsigned int dimension() {
      return m_dim;
    }
    inline double dt() {
      return m_dt;
    }
    

  private:
    void popGoal();
    void clearFirstTerm();

    void linear_interpolation();
    void calcInterpolation(InterpolationType in_itype);
    void quartic_linear();


    unsigned int m_dim;
    double m_dt;
    long unsigned int m_memsize;

    // start (pos, vel, acc)
    double *m_sx, *m_sv, *m_sa;  

    // goal points
    struct Target
    {
      double *gx, *gv, *ga;
      double time, delta;  // using delta only in QUARTIC_LINEAR
      InterpolationType itype;
    };
    std::deque<Target>  m_goals;

    // first term
    std::deque<double*> m_seqx, m_seqv, m_seqa;
  };
}

#endif
