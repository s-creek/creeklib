#include "BipedRobot.h"
#include <iostream>  // debug

using namespace creek;

BipedRobot::BipedRobot()
  : m_isInit(false)
{
  m_rsole = new Link();
  m_lsole = new Link();

  setSoleName("CREEK_RSOLE", "CREEK_LSOLE");
}


bool BipedRobot::setJointPath(const std::string &in_rfootName, const std::string &in_rfootBase, const std::string &in_lfootName, const std::string &in_lfootBase)
{
  m_isInit = true;

  // add sole
  m_robot->link(in_rfootName)->appendChild(m_rsole);
  m_robot->link(in_lfootName)->appendChild(m_lsole);
  m_robot->updateLinkTree();

  // get joint path
  m_rf2wlPath = m_robot->getJointPath(m_rsole, m_robot->link(in_rfootBase));
  m_lf2wlPath = m_robot->getJointPath(m_lsole, m_robot->link(in_lfootBase));
  m_wl2rfPath = m_robot->getJointPath(m_robot->link(in_rfootBase), m_rsole);
  m_wl2lfPath = m_robot->getJointPath(m_robot->link(in_lfootBase), m_lsole);
  m_rf2lfPath = m_robot->getJointPath(m_rsole, m_lsole);
  m_lf2rfPath = m_robot->getJointPath(m_lsole, m_rsole);
  
  return m_isInit;
}


void BipedRobot::updateWaistBase(Vector3 &in_waistPos, Matrix3 &in_waistRot)
{
  m_robot->rootLink()->p() = in_waistPos;
  m_robot->rootLink()->R() = in_waistRot;
  m_robot->calcForwardKinematics();
}


void BipedRobot::updateFootBase(Vector3 &in_footPos, Matrix3 &in_footRot, creek::FootType in_supFoot)
{
  if( in_supFoot < 0 ) {
    this->rfoot()->p() = in_footPos;
    this->rfoot()->R() = in_footRot;
    m_rf2wlPath->calcForwardKinematics();
  }
  else {
    this->lfoot()->p() = in_footPos;
    this->lfoot()->R() = in_footRot;
    m_lf2wlPath->calcForwardKinematics();
  }
  m_robot->calcForwardKinematics();
}


bool BipedRobot::calc(creek::FootType in_supportFoot, const Vector3 &in_comPosRef, const Matrix3 &in_waistRotRef,
		 const Vector3 &in_rfootPosRef, const Matrix3 &in_rfootRotRef,
		 const Vector3 &in_lfootPosRef, const Matrix3 &in_lfootRotRef)
{
  bool converged = false;
  

  // for error
  int n = m_robot->numJoints();
  std::vector<double> qorg(n);
  for(int i=0; i < n; ++i) {
    qorg[i] = m_robot->joint(i)->q();
  }
  Vector3 porg(m_robot->rootLink()->p());
  Matrix3 Rorg(m_robot->rootLink()->R());

  
  // calc inverse
  m_robot->calcForwardKinematics();
  if( in_supportFoot == creek::RFOOT ) {
    if( !m_wl2rfPath->calcInverseKinematics(in_rfootPosRef, in_rfootRotRef) )
      return false;
    
    converged = calcComInverseKinematics(in_supportFoot, in_comPosRef, in_waistRotRef, in_lfootPosRef, in_lfootRotRef);
  }
  // とりあえず両脚支持の時も左足を支持脚に
  else {
    if( !m_wl2lfPath->calcInverseKinematics(in_lfootPosRef, in_lfootRotRef) )
      return false;
    
    in_supportFoot = creek::LFOOT;
    converged = calcComInverseKinematics(in_supportFoot, in_comPosRef, in_waistRotRef, in_rfootPosRef, in_rfootRotRef);
  }

  
  if( !converged ) {
    m_robot->rootLink()->p() = porg;
    m_robot->rootLink()->R() = Rorg;
    for(int i=0; i < n; ++i) {
      m_robot->joint(i)->q() = qorg[i];
    }
    m_robot->calcForwardKinematics();
  }
  return converged;
}


bool BipedRobot::calcComInverseKinematics(creek::FootType in_supportFoot, const Vector3 &in_comPosRef, const Matrix3 &in_waistRotRef,
				     const Vector3 &in_swingPosRef, const Matrix3 &in_swingRotRef)
{
  if( !m_isInit ) {
    std::cerr << "Robot : please setJointPath()" << std::endl;
    return false;
  }
  

  //
  // set variables
  //
  static const int MAX_IK_ITERATION = 50;
  static const double LAMBDA = 0.9;


  //
  // set joint path
  //
  // sp : support,  sw : swing
  JointPathPtr sp2wlPath, sp2swPath;
  if( in_supportFoot == creek::RFOOT ) {
    sp2wlPath = m_rf2wlPath;
    sp2swPath = m_rf2lfPath;
  }
  else if( in_supportFoot == creek::LFOOT ) {
    sp2wlPath = m_lf2wlPath;
    sp2swPath = m_lf2rfPath;
  }
  else {
    return false;
  }
  Position supBase = sp2swPath->baseLink()->position();


  int n = sp2swPath->numJoints();
  std::vector<double> qorg(n);
  for(int i=0; i < n; ++i) {
    qorg[i] = sp2swPath->joint(i)->q();
  }
  

  //
  // set val conf
  //
  dmatrix J_com(3, m_robot->numJoints());
  dmatrix J_sup(6, sp2wlPath->numJoints());
  dmatrix J_leg(6, n);
  dmatrix J(12, n);
  dmatrix JJ;

#ifdef USE_CNOID_MODEL
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> QR;
  double dampingConstantSqr=1e-12;
#endif

  dvector v(12);
  dvector dq(n);

  double maxIKErrorSqr = 1.0e-16;
  double errsqr = maxIKErrorSqr * 100.0;
  bool converged = false;


  //
  // main loop
  //
  for(int k=0; k < MAX_IK_ITERATION; k++) {
    //
    // calc velocity
    //
    Vector3 comPosCur(m_robot->calcCenterOfMass());
    Vector3 dComPos(in_comPosRef-comPosCur);
    Vector3 dWaistOmega(m_robot->rootLink()->R()*model::omegaFromRot(Matrix3(m_robot->rootLink()->R().transpose()*in_waistRotRef)));
    Vector3 dSwingPos(in_swingPosRef-sp2swPath->endLink()->p());
    Vector3 dSwingOmega(sp2swPath->endLink()->R()*model::omegaFromRot(Matrix3(sp2swPath->endLink()->R().transpose()*in_swingRotRef)));

    for(int i=0; i<3; i++) {
      v(i)   = dComPos(i);
      v(i+3) = dWaistOmega(i);
      v(i+6) = dSwingPos(i);
      v(i+9) = dSwingOmega(i);
    }

#ifdef USE_CNOID_MODEL
    errsqr = v.squaredNorm();
#elif defined USE_HRP_MODEL
    errsqr = hrp::ublas::inner_prod(v,v);
#endif
    if( errsqr < maxIKErrorSqr ) {
      converged = true;
      break;
    }
    else if( MAX_IK_ITERATION == 1) {
      converged = true;
    }
    
    
    //
    // calc jacobians
    //
    Link* base = sp2swPath->baseLink();
    creek::calcCMJacobian(m_robot, base, J_com);  // calc com jacobian
    sp2wlPath->calcJacobian(J_sup);               // calc sup jacobian
    sp2swPath->calcJacobian(J_leg);               // calc legs jacobian
    

    // calc all Jacobian
    J = dzeromatrix(12,n);

    // set com jacobian
    for(int i = 0; i < sp2swPath->numJoints(); i++) {
      int id = sp2swPath->joint(i)->jointId();
      for(int j = 0; j < 3; j++) {
	J(j,i) = J_com(j,id);
      }
    }

    // set sup jacobian
    for(int i = 0; i < sp2wlPath->numJoints(); i++) {
      for(int j = 3; j < 6; j++) {
	J(j,i) = J_sup(j,i);
      } 
    }

    // set legs jacobian
    for(int i = 0; i < sp2swPath->numJoints(); i++) {
      for(int j = 0; j < 6; j++) {
	J(j+6,i) = J_leg(j,i);
      } 
    }


    //
    // solve quation
    //
#ifdef USE_CNOID_MODEL
    JJ = J *  J.transpose() + dampingConstantSqr * Eigen::MatrixXd::Identity(J.rows(), J.rows());
    dq = J.transpose() * QR.compute(JJ).solve(v);
#elif defined USE_HRP_MODEL
    model::solveLinearEquationLU(J, v, dq);
#endif

    for(int i = 0; i < n; i++) {
      sp2swPath->joint(i)->q() += LAMBDA * dq(i);
    }
    sp2swPath->baseLink()->p() = supBase.translation();
    sp2swPath->baseLink()->R() = supBase.linear();
    sp2swPath->calcForwardKinematics();
    m_robot->calcForwardKinematics();
  }


  if(!converged){
    for(int i=0; i < n; ++i){
      sp2swPath->joint(i)->q() = qorg[i];
    }
    //sp2swPath->baseLink()->position() = supBase;
    sp2swPath->baseLink()->p() = supBase.translation();
    sp2swPath->baseLink()->R() = supBase.linear();
    sp2swPath->calcForwardKinematics();
    m_robot->calcForwardKinematics();
  }
  return converged;
}

