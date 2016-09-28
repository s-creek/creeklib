#include <iostream>
#include <fstream>

#include <creeklib/interpolator/Interpolator.h>

int main()
{
  std::ofstream log("/home/grxuser/public/ogawa/workspace/src/creeklib/test/log.dat");


  creek::Interpolator inter(3, 0.05);

  double sx[] = {0, -5, 5};
  double sv[] = {1.0, 0, 0};

  double gx[] = {10, 5, -5};
  double gv[] = {0.0, 0, 0};

  double gx2[] = {5, 10, 5};

  double gx3[] = {15, 10, 5};


  inter.init(&sx[0], &sv[0]);
  inter.set(&gx[0], &gv[0], 5.0, creek::QUINTIC);
  inter.set(&gx2[0], 5.0, creek::CUBIC);
  inter.set(&gx3[0], 10.0, creek::QUARTIC_LINEAR, 1.0);


  for(int i=0; i<3; i++) {
    log << " " << sx[i];
  }
  log << std::endl;


  double outx[3], outv[3], outa[3];
  while( !inter.empty() ) {
    inter.get(&outx[0], &outv[0], &outa[0]);
    for(int i=0; i<3; i++) {
      log << " " << outx[i];
    }
    for(int i=0; i<3; i++) {
      log << " " << outv[i];
    }
    for(int i=0; i<3; i++) {
      log << " " << outa[i];
    }
    log << std::endl;

    std::cout << inter.remainingTime() << std::endl;
  }


  log.close();
  return 0;
}
