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


  // q1 -> q0
  creek::Quaternion q1(1,0,0,0);
  creek::Quaternion qs;
  qs = q1.slerp(0.9, q0);
  std::cout << qs.w() << ", " << qs.x() << ", " << qs.y() << ", " << qs.z() << std::endl;


  split();


  // Quaternion -> Rotation Matrix (3x3)
  creek::Matrix3 mat;
  mat = qs.toRotationMatrix();
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      std::cout << " " << mat(i,j);
    }
    std::cout << std::endl;
  }


  split();


  // Rotation Matrix -> Quaternion
  creek::Quaternion qm(mat);
  std::cout << qm.w() << ", " << qm.x() << ", " << qm.y() << ", " << qm.z() << std::endl;


  split();


  // check convert Quaternion <-> Rotation Matrix
  mat = qm.toRotationMatrix();
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      std::cout << " " << mat(i,j);
    }
    std::cout << std::endl;
  }


  split();


  // 逆クオータニオン
  creek::Quaternion q_inv = qm.inverse();

  // 共役クオータニオン
  creek::Quaternion q_con = qm.conjugate();

  // 回転ベクトルの長さ
  creek::Quaternion::Scalar norm = qm.norm();

  // 正規化
  creek::Quaternion qn(2,1,3,4);
  //qn.normalize();
  std::cout << qn.w() << ", " << qn.x() << ", " << qn.y() << ", " << qn.z() << std::endl;
  creek::Quaternion q_norm = qn.normalized();
  std::cout << q_norm.w() << ", " << q_norm.x() << ", " << q_norm.y() << ", " << q_norm.z() << std::endl;


  split();

  
  // template float <-> double
#ifdef USE_CNOID_MODEL
  Eigen::Quaternion<float> qf(1,2,3,4);
#elif defined USE_HRP_MODEL
  creek_tvmet::Quaternion<float> qf(1,2,3,4);
#endif
  creek::Quaternion qfd(qf);
  std::cout << qfd.w() << ", " << qfd.x() << ", " << qfd.y() << ", " << qfd.z() << std::endl;


#ifdef USE_HRP_MODEL
  split();

  qn.normalize();
  qf.normalize();
  std::cout << qn.w() << ", " << qn.x() << ", " << qn.y() << ", " << qn.z() << std::endl;
  std::cout << qf.w() << ", " << qf.x() << ", " << qf.y() << ", " << qf.z() << std::endl;

  qn = qf.slerp(0.5, qn);
  std::cout << qn.w() << ", " << qn.x() << ", " << qn.y() << ", " << qn.z() << std::endl;
#endif


  split();


  creek::Vector3 vec; vec << 0,1,0;
  creek::AngleAxis aa(0.6, vec);

  creek::Quaternion qaa(aa);
  std::cout << qaa.w() << ", " << qaa.x() << ", " << qaa.y() << ", " << qaa.z() << std::endl;


  split();


  aa = qaa;
  //aa = mat;
  std::cout << aa.angle() << std::endl;
  std::cout << aa.axis()(0) << ", " << aa.axis()(1) << ", " << aa.axis()(2) << std::endl;


  split();


  mat = aa.toRotationMatrix();
  std::cout << mat << std::endl;


  split();


  creek::AngleAxis aa2(0.4, creek::Vector3::UnitX());
  std::cout << aa2.angle() << std::endl;
  std::cout << aa2.axis()(0) << ", " << aa2.axis()(1) << ", " << aa2.axis()(2) << std::endl;


  creek::Vector3 veca, vecb;
  veca << 1,2,3;
  vecb << 2,3,4;

  double dot = veca.dot(vecb);
  std::cout << dot << std::endl;

  creek::Vector3 cross;
  cross = veca.cross(vecb);
  //cross = veca.normalized();

  std::cout << cross << std::endl; 
  std::cout << creek::Vector3::UnitY() << std::endl;


  return 0;
}
