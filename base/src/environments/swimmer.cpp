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
void SwimmerDynamics::staticEOM(Eigen::Matrix<double,2*(d+2)+1,1> state, Eigen::Matrix<double,d-1,1> action, Vector *xd) const
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

  xd->middleCols(4+d, d) = EL3.lu().solve(EL1 + EL2 + U*action).transpose();
  (*xd)[2*(d+2)] = 1.; // Time
}

void SwimmerDynamics::eom(const Vector &state, const Vector &action, Vector *xd) const
{
  if (state.size() != 2*(segments_+2)+1 || action.size() != segments_-1)
  {
    ERROR("Expected state size " << 2*(segments_+2)+1 << ", action size " << segments_-1 << ", received " << state.size() << " / " << action.size());
    throw Exception("dynamics/swimmer requires a task/swimmer subclass with equal number of segments");
  }
  
  switch (segments_)
  {
    case 2: staticEOM<2>(state.transpose(), action.transpose(), xd); break;
    case 3: staticEOM<3>(state.transpose(), action.transpose(), xd); break;
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
}

void SwimmerReachingTask::configure(Configuration &config)
{
  T_ = config["timeout"];
  randomization_ = config["randomization"];
  segments_ = config["segments"];

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
  
  Vector omin = ConstantVector(2*(segments_+2)-1, 0.),
         omax = ConstantVector(2*(segments_+2)-1, 2*M_PI);
  
  omin[0] = omin[1] = -10.;
  omax[0] = omax[1] = 10.;
  omin[2+segments_] = omin[3+segments_] = -10.;
  omax[2+segments_] = omax[3+segments_] = 10.;
  
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

void SwimmerReachingTask::start(int test, Vector *state) const
{
  *state = ConstantVector(2*(segments_+2)+1, 0.);
  
  double theta = (test?0.:randomization_)*RandGen::getUniform(0, 2*M_PI);

  (*state)[0] = 5*cos(theta);
  (*state)[1] = 5*sin(theta);
}

void SwimmerReachingTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 2*(segments_+2)+1)
    throw Exception("task/swimmer/reaching requires dynamics/swimmer with equal number of segments");

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
  obs->resize(state.size()-2);
  
  (*obs) << Tcn.transpose(), rtheta.transpose(), Vcn.transpose(), dtheta.transpose();
  
  // Shift angles to [0, 2pi], with pi in the middle
  for (size_t ii=0; ii < d-1; ++ii)
  {
    double a = fmod((*obs)[2+ii]+M_PI, 2*M_PI);
    if (a < 0) a += 2*M_PI;
    (*obs)[2+ii] = a;
  }
  
  *terminal = state[2*(d+2)] > T_;
}

void SwimmerReachingTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != 2*(segments_+2)+1 || action.size() != segments_-1 || next.size() != state.size())
    throw Exception("task/swimmer/swingup requires dynamics/swimmer with equal number of segments");

  *reward = -pow(next[0], 2) - pow(next[1], 2);
}

bool SwimmerReachingTask::invert(const Vector &obs, Vector *state) const
{
  int d = segments_;

  ColumnVector Tcn = obs.leftCols(2);
  ColumnVector rtheta = obs.middleCols(2, d-1);
  ColumnVector Vcn = obs.middleCols(2+d-1, 2);
  ColumnVector dtheta = obs.middleCols(4+d-1, d);

  ColumnVector theta(d);
  theta[0] = 0.;

  for (size_t ii=1; ii < d; ++ii)
    theta[ii] = theta[ii-1]+rtheta[ii-1]-M_PI;

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
  (*state)[2*(d+2)] = 0.;

  return true;
}

Matrix SwimmerReachingTask::rewardHessian(const Vector &state, const Vector &action) const
{
  Vector d = ConstantVector(2*(segments_+2)-1 + segments_-1, 0.);
  d[0] = d[1] = -1;
  d.middleCols(2*(segments_+2)-1, segments_-1) = ConstantVector(segments_-1, -1);

  return diagonal(d);
}
