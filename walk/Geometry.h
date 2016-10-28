// -*- c++ -*-

#ifndef CREEK_GEOMETRY_H
#define CREEK_GEOMETRY_H

#include "../util/matrixTypes.hpp"

namespace creek
{
  bool getPlaneEquation(const creek::Vector3 &in_a, const creek::Vector3 &in_b, const creek::Vector3 &in_c, creek::Vector4 &out);
  bool getPlaneEquation(const creek::Vector3 &in_p, const creek::Vector3 &in_n, creek::Vector4 &out);
  bool getIntersectPlaneAndLine(const creek::Vector4 &in_plane, const creek::Vector3 &in_A, const creek::Vector3 &in_B, creek::Vector3 &out);
  creek::Matrix3 midYaw(const creek::Matrix3 &in_a, const creek::Matrix3 &in_b);
  creek::Vector3 rpyFromRot(const creek::Matrix3 &R);
}

#endif
