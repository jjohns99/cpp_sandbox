#pragma once

#include "geometry/xform.h"
using namespace xform;
using namespace Eigen;


typedef Matrix<double, 16, 1> Vector16d;
typedef Matrix<double, 15, 1> Vector15d;
typedef Matrix<double, 12, 1> Vector12d;


struct dState
{
  Vector12d arr;
  Map<Vector3d> dpos;
  Map<Vector3d> datt;
  Map<Vector3d> dvel;
  Map<Vector3d> domega;
  dState() :
    dpos(arr.data()),
    datt(arr.data()+3),
    dvel(arr.data()+6),
    domega(arr.data()+9)
  {}

  dState operator+(const dState& x)
  {
    dState out;
    out.arr = arr + x.arr;
    return out;
  }

  dState operator*(double s)
  {
    dState out;
    out.arr = s * arr;
    return out;
  }

  dState operator/(double s)
  {
    dState out;
    out.arr = arr/s;
    return out;
  }

  dState& operator=(const dState& dx)
  {
    arr = dx.arr;
  }
};
inline dState operator*(double s, const dState x)
{
  dState out;
  out.arr = s * x.arr;
  return out;
}

struct State
{
  Vector16d arr;
  Xformd pose;
  Map<Vector3d> vel;
  Map<Vector3d> omega;
  Map<Vector3d> acc;
  State() :
    pose(arr.data()),
    vel(arr.data()+7),
    omega(arr.data()+10),
    acc(arr.data()+13)
  {}

  State& operator=(const State& x)
  {
    arr = x.arr;
  }

  State operator+(const dState& dx)
  {
    State out;
    out.pose = pose + dx.arr.segment<6>(0);
    out.vel = vel + dx.dvel;
    out.omega = omega + dx.domega;
    out.acc = acc;
    return out;
  }
};

inline double sat(double x, double max, double min)
{
  if(x > max)
    return max;
  else if(x < min)
    return min;
  else
    return x;
}

inline double max(double x, double y)
{
  return (x > y) ? x : y;
}


struct dVTOLState : public dState
{
  Vector2d dserv;
  dVTOLState() :
    dState()
  {}

  dVTOLState operator+(const dVTOLState& x)
  {
    dVTOLState out;
    out.arr = arr + x.arr;
    out.dserv = dserv + x.dserv;
    return out;
  }

  dVTOLState operator*(double s)
  {
    dVTOLState out;
    out.arr = s * arr;
    out.dserv = s * dserv;
    return out;
  }

  dVTOLState operator/(double s)
  {
    dVTOLState out;
    out.arr = arr / s;
    out.dserv = dserv / s;
    return out;
  }

  dVTOLState& operator=(const dState& dx)
  {
    arr = dx.arr;
    dserv.setZero();
  }

  dVTOLState& operator=(const dVTOLState& dx)
  {
    arr = dx.arr;
    dserv = dx.dserv;
  }
};
inline dVTOLState operator*(double s, const dVTOLState x)
{
  dVTOLState out;
  out.arr = s * x.arr;
  out.dserv = s * x.dserv;
  return out;
}


struct VTOLState: public State{
  Vector2d serv;
  VTOLState() :
    State()
  {}

  VTOLState& operator=(const State& x)
  {
    arr = x.arr;
    serv.setZero();
  }

  VTOLState& operator=(const VTOLState& x)
  {
    arr = x.arr;
    serv = x.serv;
  }

  VTOLState operator+(const dVTOLState& dx)
  {
    VTOLState out;
    out = State::operator+(dx);
    out.serv = serv + dx.dserv;
    return out;
  }
};
