#include <creeklib/walk/StepSequence.h>
#include <creeklib/walk/CapturePoint.h>
#include <iostream>
#include <fstream>
#include <cstdio>

int main()
{
  creek::StepSequence seq(0.005);
  creek::CapturePoint seq_cp(0.005);


  //
  // step
  // 
  creek::StepData start, goal;
  goal.rfoot.translation() << 0.2, 0.0, 0.05;
  goal.sup = creek::LFOOT;
  goal.double_time = 0.7;
  goal.single_time = 0.1;
    
  seq.init(start);
  seq.set(goal);
  //seq.set(goal, creek::CYCLOID_STEP);


  //
  // capture point
  //
  creek::Vector3 com, cp;
  com << 0, 0, 0.64;
  cp << 0, 0, 0;
  seq_cp.init(com, cp);
  seq_cp.calcOffset(0.19, seq.stepTime());

  double y = 0.95 - 4*seq_cp.offset();
  //std::cout << seq_cp.defaultOffset() << ", " << y << std::endl;
  cp << 0, y, 0;
  seq_cp.set(cp, seq.stepTime());


  std::ofstream log("walk.dat");
  while( !seq.empty() ) {
    //std::cout << seq.remainingTime() << ", " << seq_cp.remainingTime() << ", " << seq.numSequence() << ", " << seq_cp.numSequence() << std::endl;

    creek::StepData step;
    seq.get(step);

    creek::Vector3 zmp;
    seq_cp.get(zmp, com, cp);

    log << " " << step.rfoot.translation()(0)
	<< " " << step.rfoot.translation()(1)
	<< " " << step.rfoot.translation()(2)
	<< " " << step.sup
	<< " " << zmp(0) << " " << zmp(1)
	<< " " << com(0) << " " << com(1)
	<< " " << cp(0) << " " << cp(1)
	<< std::endl;
  }
  log.close();


  double x_d;
  creek::Vector3 ref_zmp, for_vel, y_d, cp_pre;
  for_vel << 0.25, 0, 0;
  for(int i=0; i<4; i++) {
    x_d = 0.2 + 0.4*i;
    y_d = creek::Vector3::UnitY();
    ref_zmp << x_d, -0.095, 0;
    cp_pre = cp;
    cp = seq_cp.calcNextCapturePoint(ref_zmp, y_d, for_vel, 0.8);
    ref_zmp = seq_cp.expectedZmp(cp_pre, cp, 0.8);
    std::printf("%7.4f,  %7.4f,  %7.4f,  %7.4f\n", cp(0), cp(1), ref_zmp(0), ref_zmp(1));


    x_d = 0.4 + 0.4*i;
    y_d = -creek::Vector3::UnitY();
    ref_zmp << x_d, 0.095, 0;
    cp_pre = cp;
    cp = seq_cp.calcNextCapturePoint(ref_zmp, y_d, for_vel, 0.8);
    ref_zmp = seq_cp.expectedZmp(cp_pre, cp, 0.8);
    std::printf("%7.4f,  %7.4f,  %7.4f,  %7.4f\n", cp(0), cp(1), ref_zmp(0), ref_zmp(1));
  }


  return 0;
}
