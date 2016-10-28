#include "Geometry.h"

namespace creek
{
  bool getPlaneEquation(const Vector3 &in_a, const Vector3 &in_b, const Vector3 &in_c, Vector4 &out)
  {
    Vector3 ba(in_b-in_a);
    Vector3 ca(in_c-in_a);

    Vector3 n = ba.cross(ca);
    return getPlaneEquation(in_a, n, out);
  }


  bool getPlaneEquation(const Vector3 &in_p, const Vector3 &in_n, Vector4 &out)
  {
    Vector3 n(in_n);
    if( n.norm() < 1.0e-12 )
      return false;
    if( n[2] < 0)
      n = -n;

    n.normalize();
    double d = -in_p.dot(n);

    out[0] = n[0];
    out[1] = n[1];
    out[2] = n[2];
    out[3] = d;

    return true;
  }


  // ref : http://www.hiramine.com/programming/graphics/3d_planesegmentintersection.html
  bool getIntersectPlaneAndLine(const Vector4 &in_plane, const Vector3 &in_A, const Vector3 &in_B, Vector3 &out)
  {
    Vector3 n;  n << in_plane[0], in_plane[1], in_plane[2];
    double  d = in_plane[3];
    Vector3 ab(in_B-in_A);

    if( std::fabs(n.dot(ab)) < 1.0e-12 )
      return false;

    double t = (-d - n.dot(in_A)) / n.dot(ab);
    out = in_A + t * ab;

    return true;
  }


  Matrix3 midYaw(const Matrix3 &in_a, const Matrix3 &in_b)
  {
    Vector3 rpya = rpyFromRot(in_a);
    Vector3 rpyb = rpyFromRot(in_b);

    double dyaw = rpyb[2] - rpya[2];
    while( dyaw > M_PI || dyaw < -M_PI ) {
      if( dyaw > M_PI )
	dyaw -= (2*M_PI);
      else if( dyaw < -M_PI )
	dyaw += (2*M_PI);
    }
    double yaw = rpya[2] + dyaw/2.0;
    return AngleAxis(yaw, Vector3::UnitZ()).toRotationMatrix();
  }


  // from choreonoid
  Vector3 rpyFromRot(const Matrix3 &R)
  {
    double roll, pitch, yaw;
    
    if((std::fabs(R(0,0)) < std::fabs(R(2,0))) && (std::fabs(R(1,0)) < std::fabs(R(2,0)))) {
      // cos(p) is nearly = 0
      double sp = -R(2,0);
      if (sp < -1.0) {
	sp = -1.0;
      } else if (sp > 1.0) {
	sp = 1.0;
      }
      pitch = std::asin(sp); // -pi/2< p < pi/2
            
      roll = std::atan2(sp * R(0,1) + R(1,2),  // -cp*cp*sr*cy
		   sp * R(0,2) - R(1,1)); // -cp*cp*cr*cy
            
      if (R(0,0) > 0.0) { // cy > 0
	(roll < 0.0) ? (roll += M_PI) : (roll -= M_PI);
      }
      const double sr = std::sin(roll);
      const double cr = std::cos(roll);
      if(sp > 0.0){
	yaw = std::atan2(sr * R(1,1) + cr * R(1,2), //sy*sp
		    sr * R(0,1) + cr * R(0,2));//cy*sp
      } else {
	yaw = std::atan2(-sr * R(1,1) - cr * R(1,2),
		    -sr * R(0,1) - cr * R(0,2));
      }
    } else {
      yaw = std::atan2(R(1,0), R(0,0));
      const double sa = std::sin(yaw);
      const double ca = std::cos(yaw);
      pitch = std::atan2(-R(2,0), ca * R(0,0) + sa * R(1,0));
      roll = std::atan2(sa * R(0,2) - ca * R(1,2), -sa * R(0,1) + ca * R(1,1));
    }
    Vector3 ret;  ret << roll, pitch, yaw;
    return ret;
  }
}
