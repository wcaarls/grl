/** \file lqr.cpp
 * \brief LQR solver source file.
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

#include <grl/solvers/lqr.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(LQRSolver)

void LQRSolver::request(ConfigurationRequest *config)
{
  config->push_back(CRP("operating_state", "Operating state around which to linearize", operating_state_));
  config->push_back(CRP("operating_action", "Operating action around which to linearize", operating_action_));
  config->push_back(CRP("q", "Q (state cost) matrix diagonal", q_));
  config->push_back(CRP("r", "R (action cost) matrix diagonal", r_));

  config->push_back(CRP("model", "observation_model", "Observation model", model_));
  config->push_back(CRP("policy", "policy/parameterized/state_feedback", "State feedback policy to adjust", policy_));
}

void LQRSolver::configure(Configuration &config)
{
  model_ = (ObservationModel*)config["model"].ptr();
  policy_ = (StateFeedbackPolicy*)config["policy"].ptr();
  operating_state_ = config["operating_state"];
  operating_action_ = config["operating_action"];
  q_ = config["q"];
  r_ = config["r"];
  
  if (operating_state_.empty())
    throw bad_param("solver/lqr:operating_state");

  if (operating_action_.empty())
    throw bad_param("solver/lqr:operating_action");
    
  if (q_.size() != operating_state_.size())
    throw bad_param("solver/lqr:{operating_state,q}");
  
  if (r_.size() != operating_action_.size())
    throw bad_param("solver/lqr:{operating_action,r}");
    
  if (policy_->size() != q_.size()*r_.size())
  {
    ERROR("Policy doesn't have the right size. Expected: " << q_.size()*r_.size() << ", got " << policy_->size());
    throw bad_param("solver/lqr:{policy,operating_state,operating_action}");
  }
}

void LQRSolver::reconfigure(const Configuration &config)
{
}

LQRSolver *LQRSolver::clone() const
{
  return new LQRSolver(*this);
}

void LQRSolver::solve()
{
  Eigen::VectorXd q(q_.size()), r(r_.size());
  memcpy(q.data(), q_.data(), q_.size()*sizeof(double));
  memcpy(r.data(), r_.data(), r_.size()*sizeof(double));

  Eigen::MatrixXd A = estimateStateJacobian(), At = A.transpose(),
                  B = estimateActionJacobian(), Bt = B.transpose(),
                  Q = q.asDiagonal(),
                  R = r.asDiagonal(),
                  X = Q;
                  
  if (A.size() && B.size())
  {
    CRAWL("State jacobian:\n" << A);
    CRAWL("Action jacobian:\n" << B);

    // Iterate discrete-time algebraic Riccati equation until convergence
    for (size_t ii=0; ii < 1000; ++ii)
    {
      Eigen::MatrixXd Xp = X;
      X = Q + At*X*A - At*X*B*(Bt*X*B+R).inverse()*Bt*X*A;
      if ((X - Xp).array().abs().sum() < EPS)
        break;
    } 
    
    // Compute feedback gain matrix
    Eigen::MatrixXd L = (Bt*X*B+R).inverse()*(Bt*X*A);
    
    if (std::isfinite(L.array().sum()))
    {
      CRAWL("Feedback gain matrix:\n" << L);
      
      for (size_t ii=0; ii < (size_t)L.cols(); ++ii)
        for (size_t oo=0; oo < (size_t)L.rows(); ++oo)
          policy_->params()[ii*(size_t)L.rows()+oo] = L(oo, ii);
    }
    else
      WARNING("Calculated gain matrix contains infinities");
  }
  else
    WARNING("Could not determine gain matrix");
}

Eigen::MatrixXd LQRSolver::estimateStateJacobian() const
{
  Eigen::MatrixXd J(operating_state_.size(), operating_state_.size());
  double h = 0.01;
  
  // Central differences 
  for (size_t ii=0; ii < operating_state_.size(); ++ii)
  {
    Vector state1 = operating_state_, state2 = operating_state_;
    state1[ii] -= h/2, state2[ii] += h/2;
    
    Vector res1, res2;
    double reward;
    int terminal;
    model_->step(state1, operating_action_, &res1, &reward, &terminal);
    model_->step(state2, operating_action_, &res2, &reward, &terminal);
    
    if (!res1.size() || !res2.size())
      return Eigen::MatrixXd();
    
    for (size_t jj=0; jj < operating_state_.size(); ++jj)
      J(jj, ii) = (res2[jj]-res1[jj])/h;
  }

  return J;
}

Eigen::MatrixXd LQRSolver::estimateActionJacobian() const
{
  Eigen::MatrixXd J(operating_state_.size(), operating_action_.size());
  double h = 0.01;
 
  // Central differences 
  for (size_t ii=0; ii < operating_action_.size(); ++ii)
  {
    Vector action1 = operating_action_, action2 = operating_action_;
    action1[ii] -= h/2, action2[ii] += h/2;
    
    Vector res1, res2;
    double reward;
    int terminal;
    model_->step(operating_state_, action1, &res1, &reward, &terminal);
    model_->step(operating_state_, action2, &res2, &reward, &terminal);

    if (!res1.size() || !res2.size())
      return Eigen::MatrixXd();
    
    for (size_t jj=0; jj < operating_state_.size(); ++jj)
      J(jj, ii) = (res2[jj]-res1[jj])/h;
  }
  
  return J;
}
