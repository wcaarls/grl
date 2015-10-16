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

  config->push_back(CRP("model", "observation_model", "Observation model", model_));
  config->push_back(CRP("policy", "policy/parameterized/state_feedback", "State feedback policy to adjust", policy_));
}

void LQRSolver::configure(Configuration &config)
{
  model_ = (ObservationModel*)config["model"].ptr();
  policy_ = (StateFeedbackPolicy*)config["policy"].ptr();
  operating_state_ = config["operating_state"];
  operating_action_ = config["operating_action"];
  
  if (!operating_state_.size())
    throw bad_param("solver/lqr:operating_state");

  if (!operating_action_.size())
    throw bad_param("solver/lqr:operating_action");
    
  if (policy_->size() != operating_state_.size()*operating_action_.size())
  {
    ERROR("Policy doesn't have the right size. Expected: " << operating_state_.size()*operating_action_.size() << ", got " << policy_->size());
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

bool LQRSolver::solve()
{
  Matrix J = model_->jacobian(operating_state_, operating_action_);
  Matrix H = model_->rewardHessian(operating_state_, operating_action_);
  
  if (J.size() && H.size())
  {
    Matrix Q = -H.block(0, 0, operating_state_.size(), operating_state_.size());
    Matrix R = -H.block(operating_state_.size(), operating_state_.size(), operating_action_.size(), operating_action_.size());
  
    Eigen::MatrixXd A(operating_state_.size(), operating_state_.size()), B(operating_state_.size(), operating_action_.size());
    for (size_t rr=0; rr < operating_state_.size(); ++rr)
    {
      for (size_t cc=0; cc < operating_state_.size(); ++cc)
        A(rr, cc) = J(rr, cc);
      for (size_t cc=0; cc < operating_action_.size(); ++cc)
        B(rr, cc) = J(rr, operating_state_.size()+cc);
    }

    CRAWL("State jacobian:\n" << A);
    CRAWL("Action jacobian:\n" << B);

    Eigen::MatrixXd At = A.transpose(),
                    Bt = B.transpose(),
                    X = Q;

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
    {
      WARNING("Calculated gain matrix contains infinities");
      return false;
    }
  }
  else
  {
    WARNING("Could not determine gain matrix");
    return false;
  }
    
  return true;
}
