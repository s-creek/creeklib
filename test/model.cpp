#include <iostream>
#include <cstring>

#ifdef USE_CNOID_MODEL
#include <creeklib/model/cnoidBody.hpp>
#elif defined USE_HRP_MODEL
#include <creeklib/model/hrpBody.hpp>
#endif
#include <creeklib/model/modelLoader.hpp>

void split()
{
  std::cout << std::endl << "--------------------------------------------------------------------------" << std::endl << std::endl;
}

int main(int argc, char* argv[])
{
  std::string path("/opt/grx/HRP2A/model/HRP2Amain.wrl");


  creek::BodyPtr robot( new creek::Body() );
  if( !creek::loadBody(robot, path) ) {
    std::cerr << "error : model loader" << std::endl;
    return 0;
  }
  std::cout << robot->name() << std::endl;
  std::cout << "  " << robot->numJoints() << std::endl;
  std::cout << "  " << robot->numLinks() << std::endl;


  split();


  creek::JointPathPtr arm;
  arm = robot->getJointPath(robot->link("CHEST_JOINT1"), robot->link("RARM_JOINT6"));
  for(int i=0; i<arm->numJoints(); i++)
    std::cout << "  " << arm->joint(i)->name() << std::endl;


  split();


  double half_deg[] = {15,-10,0,-30, 80,0,0};
  for(int i=0; i<arm->numJoints(); i++)
    arm->joint(i)->q() = ( half_deg[i] / 180.0 * M_PI );
  arm->calcForwardKinematics();

  std::cout << arm->endLink()->R() << std::endl;
  std::cout << arm->endLink()->R().transpose() << std::endl;


  split();


  creek::dmatrix Jcom;
  robot->calcCM();
  robot->calcCenterOfMass();
  creek::calcCMJacobian(robot, robot->link("RLEG_JOINT5"), Jcom);
  std::cout << Jcom.rows() << ", " << Jcom.cols() << std::endl;


  split();


  std::cout << robot->mass() << std::endl;


  return 0;
}
