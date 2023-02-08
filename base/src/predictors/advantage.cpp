/** \file advantage.cpp
 * \brief Advantage learning predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-10-05
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

#include <grl/predictors/advantage.h>

using namespace grl;

REGISTER_CONFIGURABLE(QPredictor)
REGISTER_CONFIGURABLE(AdvantagePredictor)

void QPredictor::request(ConfigurationRequest *config)
{
  CriticPredictor::request(config);
  
  config->push_back(CRP("alpha", "Learning rate", alpha_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Q-value representation", representation_));
  config->push_back(CRP("trace", "trace", "Trace of projections", trace_, true));
}

void QPredictor::configure(Configuration &config)
{
  CriticPredictor::configure(config);
  
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
}

void QPredictor::reconfigure(const Configuration &config)
{
  CriticPredictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

double QPredictor::criticize(const Transition &transition, const Action &action)
{
  Predictor::update(transition);

  std::vector<Vector> variants;
  std::vector<ProjectionPtr> actions;

  ProjectionPtr p = projector_->project(transition.prev_obs, transition.prev_action);
  Vector q;
  
  double target = transition.reward;
  if (transition.action.size())
  {
    discretizer_->options(transition.obs, &variants);
    projector_->project(transition.obs, variants, &actions);
    double v=-std::numeric_limits<double>::infinity();
    for (size_t kk=0; kk < variants.size(); ++kk)
      v = fmax(v, representation_->target()->read(actions[kk], &q));
      
    target += pow(gamma_, transition.tau)*v;
  }          
  double delta = target - representation_->read(p, &q);
  
  double critique = 0;
  if (action.size())  
    critique = target - representation_->read(projector_->project(transition.prev_obs, action), &q);
  
  representation_->write(p, VectorConstructor(target), alpha_);
  
  // TODO: Should clear trace on exploration
  if (trace_)
  {
    representation_->update(*trace_, VectorConstructor(alpha_*delta), pow(gamma_*lambda_, transition.tau));
    trace_->add(p, pow(gamma_*lambda_, transition.tau));
  }
  
  representation_->finalize();

  return critique;
}

// NOTE: Does not use learning rate!
LargeVector QPredictor::criticize(const std::vector<const Transition*> &transitions, const std::vector<const Action*> &actions)
{
  Matrix q, qp;
  std::vector<Vector> variants;
  std::vector<ProjectionPtr> projections;
  LargeVector critique;
  
  // Assume all states have the same number of actions
  size_t vs = discretizer_->size(transitions[0]->prev_obs);
  
  size_t qs = 0;
  for (size_t ii=0; ii < transitions.size(); ++ii)
    if (!transitions[ii]->obs.absorbing)
      qs += vs;
    
  // Get Q'(s', *)
  representation_->target()->batchRead(qs);
  for (size_t ii=0; ii < transitions.size(); ++ii)
    if (!transitions[ii]->obs.absorbing)
    {
      discretizer_->options(transitions[ii]->obs, &variants);
      projector_->project(transitions[ii]->obs, variants, &projections);
      
      for (size_t jj=0; jj < vs; ++jj)
        representation_->target()->enqueue(projections[jj]);
    }
  representation_->target()->read(&q);
    
  if (actions.size())
  {
    // Get Q(s, a)
    representation_->batchRead(transitions.size());
    for (size_t ii=0; ii < transitions.size(); ++ii)
      representation_->enqueue(projector_->project(transitions[ii]->prev_obs, *actions[ii]));
    representation_->read(&qp);
    
    critique.resize(transitions.size());
  }
  
  qs = 0;
  representation_->batchWrite(transitions.size());
  for (size_t ii=0; ii < transitions.size(); ++ii)
  {
    double target = transitions[ii]->reward;
  
    if (!transitions[ii]->obs.absorbing)
    {
      double v = q(qs++, 0);
      for (size_t kk=1; kk < vs; ++kk)
        v = fmax(v, q(qs++, 0));
      target += pow(gamma_, transitions[ii]->tau)*v;
    }
      
    representation_->enqueue(projector_->project(transitions[ii]->prev_obs, transitions[ii]->prev_action), VectorConstructor(target));
    
    if (actions.size())
      critique[ii] = target - qp(ii, 0);
  }
  representation_->write();

  return critique;  
}

void QPredictor::finalize()
{
  CriticPredictor::finalize();
  
  if (trace_)
    trace_->clear();
}

void AdvantagePredictor::request(ConfigurationRequest *config)
{
  CriticPredictor::request(config);
  
  config->push_back(CRP("alpha", "Learning rate", alpha_));
  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("lambda", "Trace decay rate", lambda_));
  config->push_back(CRP("kappa", "Advantage scaling factor", kappa_));

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "A-value representation", representation_));
  config->push_back(CRP("trace", "trace", "Trace of projections", trace_, true));
}

void AdvantagePredictor::configure(Configuration &config)
{
  CriticPredictor::configure(config);
  
  discretizer_ = (Discretizer*)config["discretizer"].ptr();

  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
  kappa_ = config["kappa"];
}

void AdvantagePredictor::reconfigure(const Configuration &config)
{
  CriticPredictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
    finalize();
}

double AdvantagePredictor::criticize(const Transition &transition, const Action &action)
{
  Predictor::update(transition);

  std::vector<Vector> variants;
  std::vector<ProjectionPtr> actions;
  Vector value;

  // A(x_t, u_t)
  ProjectionPtr p = projector_->project(transition.prev_obs, transition.prev_action);
  double a = representation_->read(p, &value);
  
  // Find max_u A(x_t, u)
  discretizer_->options(transition.prev_obs, &variants);
  projector_->project(transition.prev_obs, variants, &actions);
  double v=-std::numeric_limits<double>::infinity();
  for (size_t kk=0; kk < variants.size(); ++kk)
    v = fmax(v, representation_->read(actions[kk], &value));
    
  double target = v + (transition.reward - v)/kappa_;
  
  if (transition.action.size())
  {
    // Find max_u A(x_{t+1}, u)
    discretizer_->options(transition.obs, &variants);
    projector_->project(transition.obs, variants, &actions);
    v=-std::numeric_limits<double>::infinity();
    for (size_t kk=0; kk < variants.size(); ++kk)
      v = fmax(v, representation_->read(actions[kk], &value));
      
    target += pow(gamma_, transition.tau)*v/kappa_;
  }          
  
  double delta = target - a;
  
  representation_->write(p, VectorConstructor(target), alpha_);
  
  // TODO: Should clear trace on exploration
  if (trace_)
  {
    representation_->update(*trace_, VectorConstructor(alpha_*delta), pow(gamma_*lambda_, transition.tau));
    trace_->add(p, pow(gamma_*lambda_, transition.tau));
  }
  
  representation_->finalize();

  return target;
}

void AdvantagePredictor::finalize()
{
  CriticPredictor::finalize();
  
  if (trace_)
    trace_->clear();
}
