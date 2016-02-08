#ifndef CREEK_HRP_JOINT_PATH_HPP
#define CREEK_HRP_JOINT_PATH_HPP

#include <hrpModel/JointPath.h>
#include "hrpLink.hpp"

namespace creek
{
  class JointPath : public hrp::JointPath
  {
  public:
    JointPath() : hrp::JointPath() {}
    JointPath(hrp::Link* base, hrp::Link* end) : hrp::JointPath(base, end) {}

    inline creek::Link* joint(int index) const {
      return static_cast<creek::Link*>(hrp::JointPath::joint(index));
    }

    inline creek::Link* baseLink() const {
      return static_cast<creek::Link*>(hrp::JointPath::baseLink());
    }

    inline creek::Link* endLink() const {
      return static_cast<creek::Link*>(hrp::JointPath::endLink());
    }
  };
  typedef boost::shared_ptr<JointPath> JointPathPtr;
}

#endif
