#ifndef CREEK_HRP_LINK_HPP
#define CREEK_HRP_LINK_HPP

#include <hrpModel/Link.h>
#include <creeklib/util/matrixTypes.hpp>

namespace creek
{
  class hrpLink : public hrp::Link
  {
  public:
    hrpLink() : hrp::Link() {}
    hrpLink(const hrp::Link& link) : hrp::Link(link) {}
    
    std::string name() { return hrp::Link::name; }

    double q() const { return hrp::Link::q; }
    double& q() { return hrp::Link::q; }

    const creek::Matrix3 R() const { return creek::Matrix3(hrp::Link::R); }
    hrp::Matrix33& R() { return hrp::Link::R; }
  };
}

#endif
