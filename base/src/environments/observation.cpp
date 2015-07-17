/** \file observation.cpp
 * \brief Observation model source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-23
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

#include <grl/environments/observation.h>

using namespace grl;

REGISTER_CONFIGURABLE(FixedObservationModel)
REGISTER_CONFIGURABLE(ApproximatedObservationModel)
REGISTER_CONFIGURABLE(FixedRewardObservationModel)
 
// FixedObservationModel

void FixedObservationModel::request(ConfigurationRequest *config)
{
  config->push_back(CRP("model", "model", "Environment model", model_));
  config->push_back(CRP("task", "task", "Task to perform in the environment (should match model)", task_));
}

void FixedObservationModel::configure(Configuration &config)
{
  model_ = (Model*)config["model"].ptr();
  task_ = (Task*)config["task"].ptr();
}

void FixedObservationModel::reconfigure(const Configuration &config)
{
}

FixedObservationModel *FixedObservationModel::clone() const
{
  FixedObservationModel *om = new FixedObservationModel(*this);
  om->model_ = model_->clone();
  om->task_ = task_->clone();
  return om;
}

double FixedObservationModel::step(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal) const
{
  Vector state, next_state;
  
  if (!task_->invert(obs, &state))
  {
    ERROR("Task does not support inversion");
    next->clear();
    return 0.;
  }
  
  double tau = model_->step(state, action, &next_state);
  task_->observe(next_state, next, terminal);
  task_->evaluate(state, action, next_state, reward);
  
  return tau;
}

// ApproximatedObservationModel

void ApproximatedObservationModel::request(ConfigurationRequest *config)
{
  config->push_back(CRP("control_step", "double.control_step", "Control step time (0 = estimate using SMDP approximator)", tau_, CRP::System, 0., DBL_MAX));
  config->push_back(CRP("differential", "int.differential", "Predict state deltas", differential_, CRP::Configuration, 0, 1));
  config->push_back(CRP("wrapping", "vector.wrapping", "Wrapping boundaries", wrapping_));
  config->push_back(CRP("observation_min", "vector.observation_min", "Lower limit on observations", observation_min_, CRP::System));
  config->push_back(CRP("observation_max", "vector.observation_max", "Upper limit on observations", observation_max_, CRP::System));
  
  config->push_back(CRP("stddev_limit", "double", "Maximum standard deviation of acceptable predictions, as fraction of range", stddev_limit_, CRP::System));

  config->push_back(CRP("projector", "projector.pair", "Projector for transition model (|S|+|A| dimensions)", projector_));
  config->push_back(CRP("representation", "representation.transition", "Representation for transition model (|S|+2 dimensions)", representation_));
}

void ApproximatedObservationModel::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();

  observation_min_ = config["observation_min"];
  observation_max_ = config["observation_max"];
  
  if (observation_min_.empty() || observation_min_.size() != observation_max_.size())
    throw bad_param("observation_model/approximated:{observation_min,observation_max}");

  stddev_limit_ = config["stddev_limit"];
  
  tau_ = config["control_step"];
  
  if (tau_ == 0.)
  {
    ERROR("SMDP model approximation not supported");
    throw bad_param("observation_model/approximated:control_step");
  }
    
  differential_ = config["differential"];
  wrapping_ = config["wrapping"];
  
  if (wrapping_.empty())
    wrapping_.resize(observation_min_.size(), 0.);
    
  if (wrapping_.size() != observation_min_.size())
    throw bad_param("observation_model/approximated:wrapping");
}

void ApproximatedObservationModel::reconfigure(const Configuration &config)
{
}

ApproximatedObservationModel *ApproximatedObservationModel::clone() const
{
  ApproximatedObservationModel *om = new ApproximatedObservationModel(*this);
  om->projector_ = projector_->clone();
  om->representation_ = representation_->clone();
  return om;
}

double ApproximatedObservationModel::step(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal) const
{
  ProjectionPtr p = projector_->project(extend(obs, action)); 
  
  if (!p)
  {
    next->clear();
    return 0.;
  }
 
  Vector stddev;
  representation_->read(p, next, &stddev);
  
  if (next->empty())
    return 0.;

  *reward = (*next)[next->size()-2];
  *terminal = (*next)[next->size()-1] > 0.5;
  next->resize(next->size()-2);
  
  for (size_t ii=0; ii < obs.size(); ++ii)
  {
    if (differential_)
      (*next)[ii] += obs[ii];
    
    if (wrapping_[ii])
      (*next)[ii] = fmod(fmod((*next)[ii], wrapping_[ii]) + wrapping_[ii], wrapping_[ii]);
    
    // Don't predict starting from outside observable interval
    if (obs[ii] < observation_min_[ii] || obs[ii] > observation_max_[ii])
    {
      next->clear();
      return 0.;
    }
  }
  
  if (stddev.size())
  {
    for (size_t ii=0; ii < stddev.size()-2; ++ii)
    {
      // Don't accept inaccurate predictions
      if (stddev[ii] > stddev_limit_*(observation_max_[ii]-observation_min_[ii]))
      {
        next->clear();
        return 0.;
      }
    }
  }
  
  return tau_;
}

// FixedRewardObservationModel

void FixedRewardObservationModel::request(ConfigurationRequest *config)
{
  ApproximatedObservationModel::request(config);
  
  config->push_back(CRP("task", "task", "Task to perform in the environment", task_));
}

void FixedRewardObservationModel::configure(Configuration &config)
{
  ApproximatedObservationModel::configure(config);
  
  task_ = (Task*)config["task"].ptr();
}

void FixedRewardObservationModel::reconfigure(const Configuration &config)
{
}

FixedRewardObservationModel *FixedRewardObservationModel::clone() const
{
  FixedRewardObservationModel *om = new FixedRewardObservationModel(*this);
  om->projector_ = projector_->clone();
  om->representation_ = representation_->clone();
  om->task_ = task_->clone();
  return om;
}

double FixedRewardObservationModel::step(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal) const
{
  double tau = ApproximatedObservationModel::step(obs, action, next, reward, terminal);
  
  if (next->empty())
    return 0.;
  
  Vector state, next_state, next_obs;
  if (!task_->invert(obs, &state))
  {
    WARNING("Task does not support inversion");
    return 0.;
  }
  
  task_->invert(*next, &next_state);
  task_->evaluate(state, action, next_state, reward);
  task_->observe(next_state, &next_obs, terminal);
  
  return tau;
}
