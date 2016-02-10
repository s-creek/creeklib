#ifndef CREEK_CNOID_BODY_HPP
#define CREEK_CNOID_BODY_HPP

#ifndef USE_CNOID_MODEL
#define USE_CNOID_MODEL
#endif

#define CNOID_BACKWARD_COMPATIBILITY
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <cnoid/Jacobian>
#include "../util/matrixTypes.hpp"

namespace creek
{
  typedef cnoid::Link Link;
  typedef cnoid::JointPath JointPath;
  typedef cnoid::JointPathPtr JointPathPtr;
}

namespace creek
{
  class Body : public cnoid::Body
  {
  public:
    Body() : cnoid::Body() {};
    Body(const cnoid::Body& org) : cnoid::Body(org) {}

    inline creek::JointPathPtr getJointPath(Link* baseLink, Link* targetLink) {
      return creek::JointPathPtr(new creek::JointPath(baseLink, targetLink));
      //return cnoid::getCustomJointPath(this->clone(), baseLink, targetLink);
    }
  };
  typedef boost::shared_ptr<Body> BodyPtr;


  inline void calcCMJacobian(const cnoid::BodyPtr& body, cnoid::Link *base, dmatrix &J) {
    cnoid::calcCMJacobian(body, base, J);
  }
}

#endif
