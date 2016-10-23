#ifndef DISCRETIZATION_HPP
#define DISCRETIZATION_HPP

inline int timeToNum(double time, double dt)
{
  return ( time + 0.5*dt ) / dt;
}

#endif
