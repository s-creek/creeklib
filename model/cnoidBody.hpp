#ifndef CREEK_CNOID_BODY_HPP
#define CREEK_CNOID_BODY_HPP

#ifndef USE_CNOID_MODEL
#define USE_CNOID_MODEL
#endif

#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include "../util/matrixTypes.hpp"

namespace creek
{
  //typedef cnoid::Body Body;
  typedef cnoid::Link Link;
  typedef cnoid::JointPath JointPath;

  //typedef cnoid::BodyPtr BodyPtr;
  typedef cnoid::JointPathPtr JointPathPtr;
}

namespace creek
{
  class Body : public cnoid::Body
  {
  public:
    Body() : cnoid::Body() {};

    inline creek::JointPathPtr getJointPath(Link* baseLink, Link* targetLink) {
      return creek::JointPathPtr(new creek::JointPath(baseLink, targetLink));
    }
  };
  typedef boost::shared_ptr<Body> BodyPtr;
}

#endif
