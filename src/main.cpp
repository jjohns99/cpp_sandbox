#include <iostream>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "state.h"
// #include "geometry/xform.h"

using namespace Eigen;

Matrix3d skew_sym(Vector3d vec)
{
  Matrix3d vec_x;
  vec_x << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
  return vec_x;
}

int main()
{
  // Vector3d a;
  // Vector3d b;
  // a << 1,2,3;
  // b.setRandom();
  // double *pb = &b(0);
  // double *pa = &a(0);
  // Map<Vector3d> bmap(pb);
  // Map<Vector3d> amap(pa);
  // std::cout << b << std::endl;
  // bmap(0) += 1;
  // std::cout << b << std::endl;

  Vector4d q;
  Matrix3d R;
  q << 1,0,0,0;
  Vector3d q_bar = q.segment<3>(1);
  R = (2.0*q[0]*q[0] - 1.0)*MatrixXd::Identity(3,3) - 2*q[0]*skew(q_bar) + 2.0*q_bar*q_bar.transpose();
  Vector3d vec;
  vec << 3,4,5;
  std::cout << R*vec << std::endl;
  std::cout << q.norm() << std::endl;
  return 0;
}
