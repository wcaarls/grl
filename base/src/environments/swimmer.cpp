/** \file swimmer.cpp
 * \brief Swimmer environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-01-06
 *
 * \copyright \verbatim
 * Copyright (c) 2007, Yuval Tassa
 * Copyright (c) 2013, RLPy http://acl.mit.edu/RLPy
 * Copyright (c) 2016, Wouter Caarls
 * All rights reserved.
 *
 * This file is part of GRL, the Generic Reinforcement Learning library.
 *
 * GRL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * \endverbatim
 */

#include <grl/environments/swimmer.h>

#define ROWMULT(M, v) (((M).array().rowwise()*(v).array().transpose()).matrix())
#define COLMULT(M, v) (((M).array().colwise()*(v).array()).matrix())
#define v1Mv2(v1,M,v2) ROWMULT(COLMULT(M, v1), v2)

using namespace grl;

REGISTER_CONFIGURABLE(SwimmerDynamics)
REGISTER_CONFIGURABLE(SwimmerReachingTask)

void SwimmerDynamics::request(ConfigurationRequest *config)
{
  config->push_back(CRP("segments", "double.swimmer/segments", "Number of swimmer segments", segments_, CRP::Configuration, 2, INT_MAX));
}

void SwimmerDynamics::configure(Configuration &config)
{
  segments_ = config["segments"];
  
  const int d = segments_;
  masses_ = ConstantVector(d, 1.);
  lengths_ = ConstantVector(d, 1.);
  inertia_ = masses_.array() * lengths_.array() * lengths_.array() / 12.;
  total_mass_ = masses_.sum();

  Matrix Q = -Matrix::Identity(d, d);
  Q.topRightCorner(d-1, d-1) += Matrix::Identity(d-1, d-1);
  Q.bottomRows(1) = masses_.transpose();
  Matrix A = Matrix::Identity(d, d);
  A.topRightCorner(d-1, d-1) += Matrix::Identity(d-1, d-1);
  A(d-1, d-1) = 0.;
  
  P_ = Q.inverse()*(A*diagonal(lengths_)) / 2.;
  U_ = Matrix::Identity(d, d);
  U_.bottomLeftCorner(d-1, d-1) -= Matrix::Identity(d-1, d-1);
  U_ = U_.leftCols(d-1).eval();
  G_ = P_.transpose() * diagonal(masses_) * P_;
}

void SwimmerDynamics::reconfigure(const Configuration &config)
{
}

template<int d>
void SwimmerDynamics::staticEOM(Eigen::Matrix<double,2*(d+2)+1,1> state, Eigen::Matrix<double,d-1,1> actuation, Vector *xd) const
{
  typedef Eigen::Matrix<double,d,1> VectorD;
  typedef Eigen::Matrix<double,d,d> MatrixD;

  const double k1=7.5, k2=0.3;
  
  MatrixD P = P_, G = G_;
  Eigen::Matrix<double,d,d-1> U = U_;
  VectorD lengths = lengths_, inertia = inertia_;

  VectorD theta = state.middleRows(2, d);
  Eigen::Vector2d vcm = state.middleRows(2+d, 2);
  VectorD dtheta = state.middleRows(4+d, d);

  VectorD cth = theta.array().cos();
  VectorD sth = theta.array().sin();
  VectorD rVx = P*dtheta.cwiseProduct(-sth);
  VectorD rVy = P*dtheta.cwiseProduct(cth);
  VectorD Vx = rVx.array() + vcm[0];
  VectorD Vy = rVy.array() + vcm[1];

  VectorD Vn = Vx.cwiseProduct(-sth) + Vy.cwiseProduct(cth);
  VectorD Vt = Vx.cwiseProduct( cth) + Vy.cwiseProduct(sth);
  
  VectorD EL1 = (ROWMULT(v1Mv2(-sth, G, cth) + v1Mv2(cth, G, sth), dtheta) +
                 COLMULT(v1Mv2(cth, G, -sth) + v1Mv2(sth, G, cth), dtheta)) * dtheta;

  MatrixD DI = inertia.asDiagonal();  

  MatrixD EL3 = DI + v1Mv2(sth, G, sth) + v1Mv2(cth, G, cth);
  
  VectorD EL2 = - k1 * ROWMULT(v1Mv2(-sth, P.transpose(), -sth) + v1Mv2(cth, P.transpose(), cth), lengths) * Vn
                - k1 * (lengths.array().cube() * dtheta.array() / 12.).matrix()
                - k2 * ROWMULT(v1Mv2(-sth, P.transpose(), cth) + v1Mv2(cth, P.transpose(), sth), lengths) * Vt;

  xd->resize(state.size());
  xd->leftCols(2) = vcm.transpose();
  xd->middleCols(2, d) = dtheta.transpose();

  (*xd)[2 + d] = - (k1 * Vn.dot(-sth) + k2 * Vt.dot(cth)) / total_mass_;
  (*xd)[3 + d] = - (k1 * Vn.dot(cth) + k2 * Vt.dot(sth)) / total_mass_;

  xd->middleCols(4+d, d) = EL3.lu().solve(EL1 + EL2 + U*actuation).transpose();
  (*xd)[2*(d+2)] = 1.; // Time
}

void SwimmerDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  if (state.size() != 2*(segments_+2)+1 || actuation.size() != segments_-1)
  {
    ERROR("Expected state size " << 2*(segments_+2)+1 << ", actuation size " << segments_-1 << ", received " << state.size() << " / " << actuation.size());
    throw Exception("dynamics/swimmer requires a task/swimmer subclass with equal number of segments");
  }
  
  switch (segments_)
  {
    case 3: staticEOM<3>(state.transpose(), actuation.transpose(), xd); break;
    case 4: staticEOM<4>(state.transpose(), actuation.transpose(), xd); break;
    case 5: staticEOM<5>(state.transpose(), actuation.transpose(), xd); break;
    default:
      throw bad_param("Unsupported number of segments");
  }
}

void SwimmerReachingTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("timeout", "Episode timeout", T_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("randomization", "Level of start state randomization", randomization_, CRP::Configuration, 0., 1.));
  config->push_back(CRP("segments", "double.swimmer/segments", "Number of swimmer segments", segments_, CRP::System, 2, INT_MAX));
  config->push_back(CRP("cx", "double", "State cost factor", cx_));
  config->push_back(CRP("cu", "double", "Action cost factor", cu_));
  config->push_back(CRP("wrap_angles", "int", "Wrap angles", wrap_angles_));
}

void SwimmerReachingTask::configure(Configuration &config)
{
  T_ = config["timeout"];
  randomization_ = config["randomization"];
  segments_ = config["segments"];
  cx_ = config["cx"];
  cu_ = config["cu"];
  wrap_angles_ = config["wrap_angles"];

  const int d = segments_;
  ColumnVector masses = ConstantVector(d, 1.);
  ColumnVector lengths = ConstantVector(d, 1.);

  Matrix Q = -Matrix::Identity(d, d);
  Q.topRightCorner(d-1, d-1) += Matrix::Identity(d-1, d-1);
  Q.bottomRows(1) = masses.transpose();
  Matrix A = Matrix::Identity(d, d);
  A.topRightCorner(d-1, d-1) += Matrix::Identity(d-1, d-1);
  A(d-1, d-1) = 0.;
  P_ = Q.inverse()*(A*diagonal(lengths)) / 2.;
  
  Vector omin = ConstantVector(2*(segments_+2)-1, -M_PI),
         omax = ConstantVector(2*(segments_+2)-1,  M_PI);
  
  omin[0] = omin[1] = -20.;
  omax[0] = omax[1] = 20.;
  omin[2+segments_] = omin[3+segments_] = -20.;
  omax[2+segments_] = omax[3+segments_] = 20.;
  
  Vector amin = ConstantVector(segments_-1, -5.),
         amax = ConstantVector(segments_-1, 5.);

  config.set("observation_dims", omin.size());
  config.set("observation_min", omin);
  config.set("observation_max", omax);
  config.set("action_dims", amin.size());
  config.set("action_min", amin);
  config.set("action_max", amax);
  config.set("reward_min", -200);
  config.set("reward_max", 0);
}

void SwimmerReachingTask::reconfigure(const Configuration &config)
{
}

void SwimmerReachingTask::start(int test, Vector *state)
{
  // Randomize segment angles
  *state = Vector::Random(2*(segments_+2)+1)*(test?0.:randomization_)*M_PI;
  
  // Set initial CoM velocity to 0
  state->segment(2, 1) = 0.;
  
  // Set initial segment velocities to 0
  state->tail(segments_+1) = 0.;
  
  // Put CoM on a circle around the goal
  double theta = (test?0.:randomization_)*RandGen::getUniform(0, 2*M_PI);
  (*state)[0] = 15*cos(theta);
  (*state)[1] = 15*sin(theta);
}

void SwimmerReachingTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != 2*(segments_+2)+1)
  {
    ERROR("Received invalid state size " << state.size() << ", expected " << 2*(segments_+2)+1);
    throw Exception("task/swimmer/reaching requires dynamics/swimmer with equal number of segments");
  }

  int d = segments_;

  ColumnVector theta = state.middleCols(2, d).transpose();
  ColumnVector vcm = state.middleCols(2+d, 2).transpose();
  ColumnVector dtheta = state.middleCols(4+d, d).transpose();

  ColumnVector cth = theta.array().cos();
  ColumnVector sth = theta.array().sin();

  Matrix M = P_ - 0.5 * diagonal(ConstantVector(d, 1.));

  // stores the vector from the center of mass to the nose
  ColumnVector c2n = VectorConstructor(M.row(0)*cth, M.row(0)*sth);
  // absolute position of nose
  ColumnVector T = -state.leftCols(2).transpose().array() - c2n.array();
  // rotating coordinate such that nose is axis-aligned (nose frame)
  // (no effect when  \theta_{nose} = 0)
  ColumnVector c2n_x = VectorConstructor(cth[0], sth[0]);
  ColumnVector c2n_y = VectorConstructor(-sth[0], cth[0]);
  ColumnVector Tcn = VectorConstructor(T.dot(c2n_x), T.dot(c2n_y));

  // velocity at each joint relative to center of mass velocity
  ColumnVector vx = -M * dtheta.cwiseProduct(sth);
  ColumnVector vy = M * dtheta.cwiseProduct(cth);
  // velocity at nose (world frame) relative to center of mass velocity
  ColumnVector v2n = VectorConstructor(vx[0], vy[0]);
  // rotating nose velocity to be in nose frame
  ColumnVector Vcn = VectorConstructor((vcm+v2n).dot(c2n_x), (vcm+v2n).dot(c2n_y));

  // make angles relative
  ColumnVector rtheta = theta.bottomRows(d-1) - theta.topRows(d-1);

  // Lose one dimension because nose is axis-aligned
  // (and one because observation doesn't include time)
  obs->v.resize(state.size()-2);
  
  obs->v << Tcn.transpose(), rtheta.transpose(), Vcn.transpose(), dtheta.transpose();
  
  if (wrap_angles_)
  {
    // Make sure angles are in [-pi, pi]
    for (size_t ii=0; ii < d-1; ++ii)
    {
      double a = fmod((*obs)[2+ii]+M_PI, 2*M_PI)-M_PI;
      if (a < -M_PI) a += 2*M_PI;
      (*obs)[2+ii] = a;
    }
  }

  obs->absorbing = false;
  *terminal = state[2*(d+2)] > T_;
}

void SwimmerReachingTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  if (state.size() != 2*(segments_+2)+1 || action.size() != segments_-1 || next.size() != state.size())
    throw Exception("task/swimmer/swingup requires dynamics/swimmer with equal number of segments");
    
  double d2 = next[0]*next[0]+next[1]*next[1];
  double u2 = action.v.matrix().squaredNorm();
  
  *reward = -cx_*d2/(sqrt(d2+1))-cu_*u2;
}

bool SwimmerReachingTask::invert(const Observation &obs, Vector *state, double time) const
{
  int d = segments_;
  
  // We can't get the actual absolute position back. Just assume nose
  // is at origin and axis-aligned.

  ColumnVector Tcn = obs.v.leftCols(2);
  ColumnVector rtheta = obs.v.middleCols(2, d-1);
  ColumnVector Vcn = obs.v.middleCols(2+d-1, 2);
  ColumnVector dtheta = obs.v.middleCols(4+d-1, d);

  ColumnVector theta(d);
  theta[0] = 0.;

  for (size_t ii=1; ii < d; ++ii)
    theta[ii] = theta[ii-1]+rtheta[ii-1];

  ColumnVector cth = theta.array().cos();
  ColumnVector sth = theta.array().sin();

  Matrix M = P_ - 0.5 * diagonal(ConstantVector(d, 1.));

  // stores the vector from the center of mass to the nose
  ColumnVector c2n = VectorConstructor(M.row(0)*cth, M.row(0)*sth);

  // velocity at each joint relative to center of mass velocity
  ColumnVector vx = -M * dtheta.cwiseProduct(sth);
  ColumnVector vy = M * dtheta.cwiseProduct(cth);

  // velocity at nose (world frame) relative to center of mass velocity
  ColumnVector v2n = VectorConstructor(vx[0], vy[0]);
  
  ColumnVector cm = -Tcn - c2n;
  ColumnVector vcm = Vcn - v2n;
  
  state->resize(2*(d+2)+1);
  state->leftCols(2) = cm.transpose();
  state->middleCols(2, d) = theta;
  state->middleCols(2+d, 2) = vcm;
  state->middleCols(4+d, d) = dtheta;
  (*state)[2*(d+2)] = time;

  return true;
}

Matrix SwimmerReachingTask::rewardHessian(const Vector &state, const Action &action) const
{
  double x  = state[0], y = state[1];
  double x2 = x*x;
  double y2 = y*y;
  double d2 = x2+y2;

  double dxx = (4*cx_*x2 )/pow(d2 + 1, 1.5) - (2*cx_)/sqrt(d2 + 1) + (cx_*d2)/pow(d2 + 1, 1.5) - (3*cx_*x2 *d2)/pow(d2 + 1, 2.5);
  double dxy = (4*cx_*x*y)/pow(d2 + 1, 1.5)                                                    - (3*cx_*x*y*d2)/pow(d2 + 1, 2.5);
  double dyy = (4*cx_*y2 )/pow(d2 + 1, 1.5) - (2*cx_)/sqrt(d2 + 1) + (cx_*d2)/pow(d2 + 1, 1.5) - (3*cx_*y2 *d2)/pow(d2 + 1, 2.5);
  
  Vector d = ConstantVector(2*(segments_+2)-1 + segments_-1, 0);
  d[0] = dxx;
  d[1] = dyy;
  d.tail(segments_-1) = ConstantVector(segments_-1, -cu_);
  
  Matrix H = diagonal(d);
  H(0, 1) = dxy;
  H(1, 0) = dxy;
  
  return H;
}
