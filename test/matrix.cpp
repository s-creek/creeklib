#include <iostream>
#include <cstring>

#include <creeklib/util/matrixTypes.hpp>

void split()
{
  std::cout << std::endl << "--------------------------------------------------------------------------" << std::endl << std::endl;
}

int main()
{
  double w,x,y,z;
  double ha, angle(M_PI/180.0*60.0);
  ha = 0.5*angle;
  w = cos(ha);
  x = 0.0;
  y = sin(ha);
  z = 0.0;

  creek::Quaternion q0(w,x,y,z);
  std::cout << q0.w() << ", " << q0.x() << ", " << q0.y() << ", " << q0.z() << std::endl;


  split();


  creek::Quaternion q1(1,0,0,0);
  creek::Quaternion qs;
  qs = q1.slerp(0.9, q0);
  std::cout << qs.w() << ", " << qs.x() << ", " << qs.y() << ", " << qs.z() << std::endl;


  split();


  creek::Matrix3 mat;
  mat = qs.toRotationMatrix();
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      std::cout << " " << mat(i,j);
    }
    std::cout << std::endl;
  }


  split();


  creek::Quaternion qm(mat);
  std::cout << qm.w() << ", " << qm.x() << ", " << qm.y() << ", " << qm.z() << std::endl;


  split();


  mat = qm.toRotationMatrix();
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      std::cout << " " << mat(i,j);
    }
    std::cout << std::endl;
  }

  return 0;
}
