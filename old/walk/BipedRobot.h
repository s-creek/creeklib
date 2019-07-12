// -*- c++ -*-

#ifndef CREEK_BIPED_ROBOT_H
#define CREEK_BIPED_ROBOT_H

#include "../model/modelHeaders.h"
#include "StepSequence.h"

namespace creek
{
  class BipedRobot
  {
  public:
    BipedRobot();

    inline bool isInit() { return m_isInit; }

    inline void setRobot(creek::BodyPtr robot)  { m_robot = robot; }
    inline creek::BodyPtr body() const { return m_robot; }

    inline void setSoleName(const std::string &in_rsoleName, const std::string &in_lsoleName) {
      m_rsole->setName(in_rsoleName);
      m_lsole->setName(in_lsoleName);
    }
    inline void setSoleOffset(const creek::Vector3 &in_roffset, const creek::Vector3 &in_loffset) {
      m_rsole->setOffsetTranslation(in_roffset);
      m_lsole->setOffsetTranslation(in_loffset);
      m_robot->calcForwardKinematics();
    }
    bool setJointPath(const std::string &in_rfootName, const std::string &in_rfootBase, 
		      const std::string &in_lfootName, const std::string &in_lfootBase);

    void updateWaistBase(creek::Vector3 &in_waistPos, creek::Matrix3 &in_waistRot);
    void updateFootBase(creek::Vector3 &in_footPos, creek::Matrix3 &in_footRot, creek::FootType in_supFoot);

    // only foot
    bool calc(creek::FootType in_supportFoot, const creek::Vector3 &in_comPosRef, const creek::Matrix3 &in_waistRotRef,
	      const creek::Vector3 &in_rfootPosRef, const creek::Matrix3 &in_rfootRotRef,
	      const creek::Vector3 &in_lfootPosRef, const creek::Matrix3 &in_lfootRotRef);
    bool calcComInverseKinematics(creek::FootType in_supportFoot, const creek::Vector3 &in_comPosRef, const creek::Matrix3 &in_waistRotRef,
				  const creek::Vector3 &in_swingPosRef, const creek::Matrix3 &in_swingRotRef);
  
    inline creek::JointPathPtr rleg() const { return m_wl2rfPath; }
    inline creek::JointPathPtr lleg() const { return m_wl2lfPath; }
    inline creek::Link* rfoot() const { return m_wl2rfPath->endLink(); }
    inline creek::Link* lfoot() const { return m_wl2lfPath->endLink(); }

    
  private:
    creek::BodyPtr m_robot;
  
    bool m_isInit;
    creek::Link *m_rsole, *m_lsole;

    // wl : wasit link
    // rf : right foot,  lf : left foot
    creek::JointPathPtr m_rf2wlPath, m_lf2wlPath;
    creek::JointPathPtr m_wl2rfPath, m_wl2lfPath;
    creek::JointPathPtr m_rf2lfPath, m_lf2rfPath;
  };

  typedef boost::shared_ptr<BipedRobot> BipedRobotPtr;
}

#endif
