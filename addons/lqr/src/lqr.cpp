/** \file lqr.cpp
 * \brief LQR policy source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-26
 *
 * \copyright \verbatim
 * Copyright (c) 2015, Wouter Caarls
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

#include <grl/policies/lqr.h>

using namespace grl;

REGISTER_CONFIGURABLE(LQRPolicy)

void LQRPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("model", "observation_model", "Observation model", model_));

  config->push_back(CRP("point", "Operating point [x_0, u_0] around which to linearize", point_));
  config->push_back(CRP("q", "Q (state cost) matrix diagonal", q_));
  config->push_back(CRP("r", "R (action cost) matrix diagonal", r_));
}

void LQRPolicy::configure(Configuration &config)
{
  model_ = (ObservationModel*)config["model"].ptr();
  point_ = config["point"];
  q_ = config["q"];
  r_ = config["r"];
  
  if (q_.size() + r_.size() != point_.size())
  {
    ERROR("Expected operating point size " << q_.size() << " + " << r_.size() << ", got " << point_.size());
    throw bad_param("policy/lqr:{point,q,r}");
  }
    
  Eigen::VectorXd q(q_.size()), r(r_.size());
  memcpy(q.data(), q_.data(), q_.size()*sizeof(double));
  memcpy(r.data(), r_.data(), r_.size()*sizeof(double));

  DEBUG("Computing jacobians");
    
  Eigen::MatrixXd A = getStateJacobian(model_, point_), At = A.transpose(),
                  B = getActionJacobian(model_, point_), Bt = B.transpose(),
                  Q = q.asDiagonal(),
                  R = r.asDiagonal(),
                  X = Q;

  CRAWL("State jacobian:\n" << A);
  CRAWL("Action jacobian:\n" << B);

  DEBUG("Computing feedback gains");

  // Iterate discrete-time algebraic Riccati equation until convergence
  while (true)
  {
    Eigen::MatrixXd Xp = X;
    X = Q + At*X*A - At*X*B*(Bt*X*B+R).inverse()*Bt*X*A;
    if ((X - Xp).array().abs().sum() < 0.001)
      break;
  } 
  
  // Compute feedback gain matrix
  L_ = (Bt*X*B+R).inverse()*(Bt*X*A);
  
  CRAWL("Feedback gain matrix:\n" << L_);
}

void LQRPolicy::reconfigure(const Configuration &config)
{
}

LQRPolicy *LQRPolicy::clone() const
{
  return new LQRPolicy(*this);
}

void LQRPolicy::act(const Vector &in, Vector *out) const
{
  if (in.size() != (size_t)L_.cols())
  {
    ERROR("Expected input size " << L_.cols() << ", got " << in.size());
    throw bad_param("policy/lqr:{model,point,q}");
  }

  Eigen::VectorXd x(in.size());
  for (size_t ii=0; ii < in.size(); ++ii)
    x[ii] = in[ii] - point_[ii];

  Eigen::VectorXd u = -L_*x;
  
  out->resize(u.size());
  for (size_t ii=0; ii < out->size(); ++ii)
    (*out)[ii] = u[ii] + point_[q_.size()+ii];
}

Eigen::MatrixXd LQRPolicy::getStateJacobian(const ObservationModel *model, const Vector &point) const
{
  Vector state(q_.size()), action(r_.size());
  for (size_t ii=0; ii < q_.size(); ++ii)
    state[ii] = point[ii];
  for (size_t ii=0; ii < r_.size(); ++ii)
    action[ii] = point[q_.size()+ii];
  
  Eigen::MatrixXd J(state.size(), state.size());
  double h = 0.0001;
  
  // Central differences 
  for (size_t ii=0; ii < state.size(); ++ii)
  {
    Vector state1 = state, state2 = state;
    state1[ii] -= h/2, state2[ii] += h/2;
    
    Vector res1, res2;
    double reward;
    int terminal;
    model->step(state1, action, &res1, &reward, &terminal);
    model->step(state2, action, &res2, &reward, &terminal);
    
    for (size_t jj=0; jj < state.size(); ++jj)
      J(jj, ii) = (res2[jj]-res1[jj])/h;
  }

  return J;
}

Eigen::MatrixXd LQRPolicy::getActionJacobian(const ObservationModel *model, const Vector &point) const
{
  Vector state(q_.size()), action(r_.size());
  for (size_t ii=0; ii < q_.size(); ++ii)
    state[ii] = point[ii];
  for (size_t ii=0; ii < r_.size(); ++ii)
    action[ii] = point[q_.size()+ii];
  
  Eigen::MatrixXd J(state.size(), action.size());
  double h = 0.0001;
 
  // Central differences 
  for (size_t ii=0; ii < action.size(); ++ii)
  {
    Vector action1 = action, action2 = action;
    action1[ii] -= h/2, action2[ii] += h/2;
    
    Vector res1, res2;
    double reward;
    int terminal;
    model->step(state, action1, &res1, &reward, &terminal);
    model->step(state, action2, &res2, &reward, &terminal);
    
    for (size_t jj=0; jj < state.size(); ++jj)
      J(jj, ii) = (res2[jj]-res1[jj])/h;
  }
  
  return J;
}
