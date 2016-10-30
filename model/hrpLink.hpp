#ifndef CREEK_HRP_LINK_HPP
#define CREEK_HRP_LINK_HPP

#include <hrpModel/Link.h>
#include "../util/matrixTypes.hpp"

namespace creek
{
  class Link : public hrp::Link
  {
  public:
    Link() : hrp::Link() {}
    Link(const hrp::Link& link) : hrp::Link(link) {}
    
    std::string name() { return hrp::Link::name; }
    void setName(const std::string &set_name)  { hrp::Link::name = set_name; }

    inline double q() const { return hrp::Link::q; }
    inline double& q() { return hrp::Link::q; }

    inline const creek::Matrix3 R() const { return creek::Matrix3(hrp::Link::R); }
    inline creek::Matrix3& R() { return static_cast<creek::Matrix3&>(hrp::Link::R); }

    inline const creek::Vector3 p() const { return creek::Vector3(hrp::Link::p); }
    inline creek::Vector3& p() { return static_cast<creek::Vector3&>(hrp::Link::p); }

    inline const creek::Position position() const {
      creek::Position ret;
      ret.translation() = this->p();
      ret.linear() = this->R();
      return ret;
    }

    inline void appendChild(hrp::Link *link) { this->addChild(link); }

    inline void setOffsetPosition(const creek::Position &offset) {
      setOffsetTranslation(offset.translation());
      setOffsetRotation(offset.linear());
    }
    inline void setOffsetTranslation(const creek::Vector3 &offset) { this->b = offset; }
    inline void setOffsetRotation(const creek::Matrix3 &offset) { this->Rs = offset; }
  };
}

#endif
