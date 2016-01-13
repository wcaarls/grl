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

void ObservationModel::request(ConfigurationRequest *config)
{
  config->push_back(CRP("jacobian_step", "double", "Step size for Jacobian estimation", jacobian_step_, CRP::Online, DBL_MIN, DBL_MAX));
}

void ObservationModel::configure(Configuration &config)
{
  jacobian_step_ = config["jacobian_step"];
}

void ObservationModel::reconfigure(const Configuration &config)
{
  config.get("jacobian_step", jacobian_step_);
}

Matrix ObservationModel::jacobian(const Vector &obs, const Vector &action) const
{
  Matrix J(obs.size()+1, obs.size()+action.size());
  
  // Central differences 
  for (size_t ii=0; ii < obs.size()+action.size(); ++ii)
  {
    Vector res1, res2;
    double reward1, reward2;
    int terminal;
    
    if (ii < obs.size())
    {
      // d obs / d obs
      Vector state1 = obs, state2 = obs;
      state1[ii] -= jacobian_step_/2; state2[ii] += jacobian_step_/2;
    
      step(state1, action, &res1, &reward1, &terminal);
      step(state2, action, &res2, &reward2, &terminal);
    }
    else
    {
      // d obs / d action
      Vector action1 = action, action2 = action;
      action1[ii-obs.size()] -= jacobian_step_/2; action2[ii-obs.size()] += jacobian_step_/2;
      
      step(obs, action1, &res1, &reward1, &terminal);
      step(obs, action2, &res2, &reward2, &terminal);
    }
    
    if (!res1.size() || !res2.size())
      return Matrix();
    
    for (size_t jj=0; jj < obs.size(); ++jj)
      J(jj, ii) = (res2[jj]-res1[jj])/jacobian_step_;
    J(obs.size(), ii) = (reward2-reward1)/jacobian_step_;
  }

  return J;
}

Matrix ObservationModel::rewardHessian(const Vector &obs, const Vector &action) const
{
  Matrix H(obs.size()+action.size(), obs.size()+action.size());
  
  // Central differences 
  for (size_t ii=0; ii < obs.size()+action.size(); ++ii)
  {
    double grad1, grad2;
    Vector obs1=obs, obs2=obs, action1=action, action2=action;
  
    if (ii < obs.size())
    {
      obs1[ii] -= jacobian_step_/2;
      obs2[ii] += jacobian_step_/2;
    }
    else
    {
      action1[ii-obs.size()] -= jacobian_step_/2;
      action2[ii-obs.size()] += jacobian_step_/2;
    }
    
    Matrix J1 = jacobian(obs1, action1), J2 = jacobian(obs2, action2);
    
    if (!J1.size() || !J2.size())
      return Matrix();
  
    for (size_t jj=ii; jj < obs.size()+action.size(); ++jj)
      H(ii, jj) = H(jj, ii) = (J2(obs.size(), jj)-J1(obs.size(), jj))/jacobian_step_;
  }    

  return H;
}
 
// FixedObservationModel

void FixedObservationModel::request(ConfigurationRequest *config)
{
  ObservationModel::request(config);

  config->push_back(CRP("model", "model", "Environment model", model_));
  config->push_back(CRP("task", "task", "Task to perform in the environment (should match model)", task_));
}

void FixedObservationModel::configure(Configuration &config)
{
  ObservationModel::configure(config);
  
  model_ = (Model*)config["model"].ptr();
  task_ = (Task*)config["task"].ptr();
}

void FixedObservationModel::reconfigure(const Configuration &config)
{
  ObservationModel::reconfigure(config);
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
    *next = Vector();
    return 0.;
  }
  
  double tau = model_->step(state, action, &next_state);
  task_->observe(next_state, next, terminal);
  task_->evaluate(state, action, next_state, reward);
  
  return tau;
}

Matrix FixedObservationModel::rewardHessian(const Vector &obs, const Vector &action) const
{
  Matrix H = task_->rewardHessian(obs, action);
  if (!H.size())
    return ObservationModel::rewardHessian(obs, action);
  else
    return H;
}

// ApproximatedObservationModel

void ApproximatedObservationModel::request(ConfigurationRequest *config)
{
  ObservationModel::request(config);

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
  ObservationModel::configure(config);

  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();

  observation_min_ = config["observation_min"];
  observation_max_ = config["observation_max"];
  
  if (!observation_min_.size() || observation_min_.size() != observation_max_.size())
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
  
  if (!wrapping_.size())
    wrapping_ = ConstantVector(observation_min_.size(), 0.);
    
  if (wrapping_.size() != observation_min_.size())
    throw bad_param("observation_model/approximated:wrapping");
}

void ApproximatedObservationModel::reconfigure(const Configuration &config)
{
  ObservationModel::reconfigure(config);
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
    *next = Vector();
    return 0.;
  }
 
  Vector pred, stddev;
  representation_->read(p, &pred, &stddev);
  
  if (!pred.size())
    return 0.;

  *reward = pred[pred.size()-2];
  *terminal = 2*(pred[pred.size()-1] > 0.5);
  next->resize(pred.size()-2);
  for (size_t ii=0; ii < next->size(); ++ii)
    (*next)[ii] = pred[ii];
  
  for (size_t ii=0; ii < obs.size(); ++ii)
  {
    if (differential_)
      (*next)[ii] += obs[ii];
    
    if (wrapping_[ii])
      (*next)[ii] = fmod(fmod((*next)[ii], wrapping_[ii]) + wrapping_[ii], wrapping_[ii]);
    
    // Don't predict starting from outside observable interval
    if (obs[ii] < observation_min_[ii] || obs[ii] > observation_max_[ii])
    {
      *next = Vector();
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
        *next = Vector();
        return 0.;
      }
    }
  }
  
  return tau_;
}

Matrix ApproximatedObservationModel::jacobian(const Vector &obs, const Vector &action) const
{
  // Get complete Jacobian.
  Matrix J = representation_->jacobian(projector_->project(obs, action))*projector_->jacobian(extend(obs, action));
  
  // If representation doesn't support closed-form Jacobians, use estimation.
  if (!J.size())
    return ObservationModel::jacobian(obs, action);
  
  // Return only the rows relating to the state and reward.
  Matrix rJ(obs.size()+1, J.cols());
  for (size_t rr=0; rr < rJ.rows(); ++rr)
    for (size_t cc=0; cc < rJ.cols(); ++cc)
      rJ(rr, cc) = J(rr, cc);

  // If model is differential, add identity matrix to state rows
  if (differential_)
    for (size_t ii=0; ii < rJ.rows()-1; ++ii)
      rJ(ii, ii) += 1;
      
  return rJ;
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
  ApproximatedObservationModel::reconfigure(config);
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
  
  if (!next->size())
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

Matrix FixedRewardObservationModel::rewardHessian(const Vector &obs, const Vector &action) const
{
  Matrix H = task_->rewardHessian(obs, action);
  if (!H.size())
    return ApproximatedObservationModel::rewardHessian(obs, action);
  else
    return H;
}
