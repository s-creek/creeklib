#define USE_CNOID_MODEL
#include "../model/modelLoader.hpp"
#include "WalkPlanner.h"
#include "CapturePoint.h"
#include "BipedRobot.h"

#include <iostream>
#include <fstream>

int main()
{
 //
  // setup robot
  //
  //std::string path("/home/player/tsml/model/JVRC-1/main.wrl");
  std::string path("/opt/grx/HRP2A/model/HRP2Amain.wrl");
  creek::BodyPtr robot( new creek::Body() );
  if( !creek::loadBody(robot, path) ) {
    std::cerr << "error : model loader" << std::endl;
    return 0;
  }
  

  creek::BipedRobotPtr biped( new creek::BipedRobot() );
  biped->setRobot(robot);

  //biped->setJointPath("R_ANKLE_P", "PELVIS", "L_ANKLE_P", "PELVIS");
  biped->setJointPath("RLEG_JOINT5", "WAIST", "LLEG_JOINT5", "WAIST");
  robot->rootLink()->p() << 0.0, 0.0, 0.854;

  creek::Vector3 offset;  offset << 0, 0, -0.108;
  biped->setSoleOffset(offset, offset);


  //
  // half sitting
  //
  biped->body()->rootLink()->p() << -0.02, 0.0, 0.790861;
  double qDeg[] = {-25.6, 0, 0, 57.8, 0, -32.2,
		   -25.6, 0, 0, 57.8, 0, -32.2,
		   0, 0, 0,
		   0, 0, 0,
		   10, -5, 0, -30, 0, 0, 0,
		   0, 0, 0, 0, 0, 0,
		   10,  5, 0, -30, 0, 0, 0,
		   0, 0, 0, 0, 0, 0};
  for(int i=0; i<robot->numJoints(); i++)
    robot->joint(i)->q() = qDeg[i] / 180.0 * M_PI;
  robot->calcForwardKinematics();

  std::cout << biped->rfoot()->p()(2) << std::endl;
  std::cout << biped->lfoot()->p()(2) << std::endl;


  //
  // walk planner
  //
  creek::WalkPlanner *walk = new creek::WalkPlanner(0.005);
  walk->setTime(0.7, 0.1);
  walk->setFootSize(0.160, 0.100, 0.055, 0.055);
  walk->init(biped);
  walk->calcOffset(0.19, 0.8, walk->capturePoint()->comHeight());
  

  //
  // test
  //
  walk->setStepping(true);
  if( walk->empty() ) {
    creek::FootType swing_foot_type;
    creek::StepData step;
    walk->getGoal(step);
   
   
    // add step
    if( walk->isStepping() ) {
      step.sup = creek::reverseFoot(step.sup);
      if( step.sup == creek::DFOOT )
	step.sup = creek::LFOOT;

      step.stepType = creek::CYCLOID_STEP;
      walk->addStep(step);

      step.sup = creek::reverseFoot(step.sup);
      walk->addStep(step);
    }
    else if( (swing_foot_type = walk->getSwingFootType(step.rfoot, step.lfoot)) != creek::AIR ) {
      switch(swing_foot_type)
	{
	case creek::RFOOT:
	case creek::LFOOT:
	  {
	    step.sup = creek::reverseFoot(swing_foot_type);
	    walk->addStep(step);
	  }
	  break;
	case creek::DFOOT:
	default:
	  walk->stop();
	}
    }
    else {
      return 0;
    }
  }
  walk->stop();

  std::ofstream log("log.dat");
  while( !walk->empty() ) {
    creek::StepData step;
    walk->get(step);
    log << " " << step.rfoot.translation()(0) << " " << step.rfoot.translation()(1) << " " << step.rfoot.translation()(2)
	<< " " << step.lfoot.translation()(0) << " " << step.lfoot.translation()(1) << " " << step.lfoot.translation()(2)
	<< " " << step.zmp(0) << " " << step.zmp(1) << " " << step.zmp(2)
	<< " " << step.com(0) << " " << step.com(1) << " " << step.com(2)
	<< " " << step.cp(0) << " " << step.cp(1) << " " << step.cp(2)
	<< " " << step.sup
	<< std::endl;
  }


  return 0;
}
