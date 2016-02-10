#ifndef CREEK_HRP_BODY_HPP
#define CREEK_HRP_BODY_HPP

#ifndef USE_HRP_MODEL
#define USE_HRP_MODEL
#endif

#include <hrpModel/Body.h>
#include "hrpLink.hpp"
#include "hrpJointPath.hpp"

namespace creek
{
  class Body : public hrp::Body
  {
  public:
    Body() : hrp::Body() {}

    inline creek::Link* joint(int id) const {
      return static_cast<creek::Link*>(hrp::Body::joint(id));
    }

    inline creek::Link* link(int index) const {
      return static_cast<creek::Link*>(hrp::Body::link(index));
    }

    inline creek::Link* link(const std::string& name) const {
      return static_cast<creek::Link*>(hrp::Body::link(name));
    }

    inline creek::Link* rootLink() const {
      return static_cast<creek::Link*>(hrp::Body::rootLink());
    }

    inline creek::JointPathPtr getJointPath(hrp::Link* baseLink, hrp::Link* targetLink) {
      //return creek::JointPathPtr(new creek::JointPath(baseLink, targetLink));
      return boost::static_pointer_cast<creek::JointPath>( hrp::Body::getJointPath(baseLink, targetLink) );
    }
  };

  typedef boost::intrusive_ptr<Body> BodyPtr;


  inline void calcCMJacobian(const hrp::BodyPtr& body, hrp::Link *base, dmatrix &J) {
    body->calcCMJacobian(base, J);
  }
}

#endif
