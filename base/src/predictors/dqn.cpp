/** \file dqn.cpp
 * \brief DQN predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2023-02-02
 *
 * \copyright \verbatim
 * Copyright (c) 2023, Wouter Caarls
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

#include <grl/predictors/dqn.h>

using namespace grl;

REGISTER_CONFIGURABLE(DQNPredictor)

void DQNPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);
  
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("double", "Explicitly read target policy's value from target representation", double_, CRP::Configuration, 0, 1));

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.state", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/actions", "Action Q-vector representation", representation_));
  config->push_back(CRP("policy", "mapping/policy/discrete/value", "Value based target policy", policy_));
}

void DQNPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  policy_ = (ValuePolicy*)config["policy"].ptr();
  
  gamma_ = config["gamma"];
  double_ = config["double"];
}

void DQNPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

void DQNPredictor::update(const std::vector<const Transition*> &transitions)
{
  Matrix qp;
  LargeVector v;
  
  // Get Q(s, *)
  representation_->batchRead(transitions.size());
  for (size_t ii=0; ii < transitions.size(); ++ii)
    representation_->enqueue(projector_->project(transitions[ii]->prev_obs));
  representation_->read(&qp);
  
  size_t actions = discretizer_->size(transitions[0]->prev_action);
  if (qp.cols() != actions)
  {
    ERROR("Representation has " << qp.cols() << " actions, but discretizer requires " << actions);
    throw bad_param("predictor/dqn:{discretizer,representation}");
  }

  std::vector<const Observation*> observations(transitions.size());
  std::vector<const Action*> prev_actions(transitions.size());
  for (size_t ii=0; ii < transitions.size(); ++ii)
  {
    observations[ii] = &transitions[ii]->obs;
    prev_actions[ii] = &transitions[ii]->prev_action;
  }
  
  if (double_)
  {
    Matrix q, pi;
  
    // Get pi(s', *)
    // Note: given policy must NOT use target representation, otherwise it is normal but less efficient DQN
    policy_->distribution(observations, prev_actions, &pi);
    
    // Get Q'(s', *)
    representation_->target()->batchRead(transitions.size());
    for (size_t ii=0; ii < transitions.size(); ++ii)
      representation_->target()->enqueue(projector_->project(transitions[ii]->obs));
    representation_->target()->read(&q);
    
    // Get E_pi(s') Q'(s', a')
    v = (q.array() * pi.array()).rowwise().sum();
  }
  else
  {
    // Get E_pi'(s') Q'(s', a')
    // Note: given policy must use target representation
    policy_->value(observations, prev_actions, &v);
  }

  // Calculate targets: Q(s, a) <- r + gamma*V'(s') for taken a
  representation_->batchWrite(transitions.size());
  for (size_t ii=0; ii < transitions.size(); ++ii)
  {
    double target = transitions[ii]->reward;
  
    if (!transitions[ii]->obs.absorbing)
      target += pow(gamma_, transitions[ii]->tau)*v[ii];
      
    qp(ii, discretizer_->discretize(transitions[ii]->prev_action)) = target;
    representation_->enqueue(projector_->project(transitions[ii]->prev_obs), qp.row(ii));
  }
  representation_->write();
}

void DQNPredictor::finalize()
{
  Predictor::finalize();
}
