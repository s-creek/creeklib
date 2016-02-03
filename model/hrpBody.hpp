#ifndef CREEK_HRP_BODY_HPP
#define CREEK_HRP_BODY_HPP

#include <hrpModel/Body.h>
#include "hrpLink.hpp"

namespace creek
{
  class hrpBody : public hrp::Body
  {
  public:
    hrpBody() {}

    inline hrpLink* joint(int id) {
      return static_cast<hrpLink*>(hrp::Body::joint(id));
    }
  };

  typedef boost::intrusive_ptr<hrpBody> hrpBodyPtr;
}

#endif
