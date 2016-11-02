#include <iostream>
#include <fstream>

#include <creeklib/interpolator/RotationInterpolator.h>

int main()
{
  creek::RotationInterpolator rinter(0.5);

  creek::Matrix3 sR(creek::Matrix3::Identity());
  creek::Matrix3 gR;
  gR = creek::AngleAxis(M_PI, creek::Vector3::UnitZ()).toRotationMatrix();

  rinter.init(sR);
  rinter.set(gR, 10.0);


  creek::Matrix3 out;
  while( !rinter.empty() ) {
    rinter.get(out);
    std::cout << out << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;
  }


  return 0;
}
