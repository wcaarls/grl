/** \file modeled.cpp
 * \brief Modeled environment and dynamical model source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#include <grl/environment.h>

using namespace grl;

REGISTER_CONFIGURABLE(ModeledEnvironment)
REGISTER_CONFIGURABLE(DynamicalModel)

void ModeledEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("discrete_time", "Always report unit step time", discrete_time_, CRP::Configuration, 0, 1));
  config->push_back(CRP("window", "Number of observations to concatenate", window_, CRP::Configuration, 1, 10));
  config->push_back(CRP("delta", "Action delta for differential actions", delta_, CRP::Configuration));

  config->push_back(CRP("model", "model", "Environment model", model_));
  config->push_back(CRP("task", "task", "Task to perform in the environment (should match model)", task_));
  config->push_back(CRP("exporter", "exporter", "Optional exporter for transition log (supports time, state, observation, action, reward, terminal)", exporter_, true));

  config->push_back(CRP("state", "signal/vector.state", "Current state of the model", CRP::Provided));
  config->push_back(CRP("action", "signal/vector.action", "Last action applied to the model", CRP::Provided));
  
  config->push_back(CRP("observation_dims", "int.observation_dims", "Number of observation dimensions, taking window into account", CRP::Provided));
  config->push_back(CRP("observation_min", "vector.observation_min", "Lower limit on observations, taking window into account", CRP::Provided));
  config->push_back(CRP("observation_max", "vector.observation_max", "Upper limit on observations, taking window into account", CRP::Provided));
  config->push_back(CRP("action_dims", "int.action_dims", "Number of action dimensions", CRP::Provided));
  config->push_back(CRP("action_min", "vector.action_min", "Lower limit on actions", CRP::Provided));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit on actions", CRP::Provided));
  config->push_back(CRP("reward_min", "vector.reward_min", "Lower limit on immediate reward", CRP::Provided));
  config->push_back(CRP("reward_max", "vector.reward_max", "Upper limit on immediate reward", CRP::Provided));
}

void ModeledEnvironment::configure(Configuration &config)
{
  discrete_time_ = config["discrete_time"];
  window_ = config["window"];
  delta_ = config["delta"].v(); 
  model_ = (Model*)config["model"].ptr();
  task_ = (Task*)config["task"].ptr();
  exporter_ = (Exporter*)config["exporter"].ptr();
  
  // Register fields to be exported
  if (exporter_)
    exporter_->init({"time", "state", "observation", "action", "reward", "terminal"});
  
  state_obj_ = new VectorSignal();
  action_obj_ = new VectorSignal();
  
  config.set("state", state_obj_);
  config.set("action", action_obj_);
  
  // Get child configuration
  Vector observation_min = (*this)["task/observation_min"].v().replicate(1, window_);
  Vector observation_max = (*this)["task/observation_max"].v().replicate(1, window_);
  
  action_min_ = (*this)["task/action_min"].v();
  action_max_ = (*this)["task/action_max"].v();
  
  Vector action_min = action_min_;
  Vector action_max = action_max_;
  
  // Extend observation with action when using differential actions
  if (delta_.size())
  {
   // Using differential actions
    if (delta_.size() != action_min.size())
      throw bad_param("environment/modeled:delta");
  
    // Extend observation vectors with past action
    observation_min = extend(observation_min, action_min);
    observation_max = extend(observation_max, action_max);
    
    // Modify action vectors to allowed differential range
    action_min = -delta_;
    action_max = delta_;
  }
  
  config.set("observation_dims", observation_min.size());
  config.set("observation_min", observation_min);
  config.set("observation_max", observation_max);
  
  // Forward other task parameters
  config.set("action_dims", action_min.size());
  config.set("action_min", action_min);
  config.set("action_max", action_max);
  config.set("reward_min", (double)(*this)["task/reward_min"]);
  config.set("reward_max", (double)(*this)["task/reward_max"]);
}

void ModeledEnvironment::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    time_learn_ = time_test_ = jerk_ = 0.;
}

ModeledEnvironment &ModeledEnvironment::copy(const Configurable &obj)
{
  const ModeledEnvironment& me = dynamic_cast<const ModeledEnvironment&>(obj);
  
  obs_ = me.obs_;
  test_ = me.test_;
  
  return *this;
}
    
void ModeledEnvironment::start(int test, Observation *obs)
{
  int terminal;

  task_->start(test, &state_);
  Observation this_obs;
  task_->observe(state_, &this_obs, &terminal);
  *obs = this_obs;
  obs->v = ConstantVector(this_obs.size()*window_ + action_min_.size()*(delta_.size()>0), 0.);
  obs->v.segment(this_obs.size()*(window_-1), this_obs.size()) = this_obs.v;
    
  actuation_ = Vector();
  jerk_ = 0;

  obs_ = *obs;
  state_obj_->set(state_);
  
  test_ = test;

  if (exporter_)
    exporter_->open((test_?"test":"learn"), (test_?time_test_:time_learn_) != 0.0);
}

double ModeledEnvironment::step(const Action &action, Observation *obs, double *reward, int *terminal)
{
  Vector state = state_, next, actuation;
  double tau = 0;
  bool done;
  
  Action _action = action;
  if (delta_.size())

    _action.v = (obs_.v.tail(action_min_.size())+action.v).min(action_max_).max(action_min_);

  task_->actuate(state_, state, _action, &actuation);
  if (!actuation_.size())
    actuation_ = actuation;

  do
  {
    tau += model_->step(state, actuation, &next);
    
    jerk_ += (actuation-actuation_).matrix().norm()*tau;
    actuation_ = actuation;
    
    state = next;
    done = task_->actuate(state_, state, _action, &actuation);
  } while (!done);
  
  Observation this_obs;
  task_->observe(next, &this_obs, terminal);
  obs_.v.head(this_obs.size()*(window_-1)) = obs_.v.segment(this_obs.size(), this_obs.size()*(window_-1));
  obs_.v.segment(this_obs.size()*(window_-1), this_obs.size()) = this_obs.v;
  if (delta_.size())
    obs_.v.tail(delta_.size()) = _action.v;
  obs_.u = action.v;
  *obs = obs_;

  task_->evaluate(state_, _action, next, reward);

  double &time = test_?time_test_:time_learn_;
  
  if (exporter_)
    exporter_->write({VectorConstructor(time), state_, *obs, action, VectorConstructor(*reward), VectorConstructor((double)*terminal)});

  time += tau;

  state_ = next;
  state_obj_->set(state_);
  action_obj_->set(_action);

  if (discrete_time_)
    return 1;
  else  
    return tau;
}

void ModeledEnvironment::report(std::ostream &os) const
{
  os << std::setw(15) << time_learn_ << std::setw(15) << jerk_;
  model_->report(os, state_);
  task_->report(os, state_);
}

void DynamicalModel::request(ConfigurationRequest *config)
{
  config->push_back(CRP("control_step", "double.control_step", "Control step time", tau_, CRP::Configuration, 0.00001, DBL_MAX));
  config->push_back(CRP("integration_steps", "Number of integration steps per control step", (int)steps_, CRP::Configuration, 1));

  config->push_back(CRP("dynamics", "dynamics", "Equations of motion", dynamics_));
}

void DynamicalModel::configure(Configuration &config)
{
  dynamics_ = (Dynamics*)config["dynamics"].ptr();
  
  tau_ = config["control_step"];
  steps_ = config["integration_steps"];
}

void DynamicalModel::reconfigure(const Configuration &config)
{
}

double DynamicalModel::step(const Vector &state, const Vector &actuation, Vector *next) const
{
  Vector xd;
  double h = tau_/steps_;
  
  *next = state;
  
  for (size_t ii=0; ii < steps_; ++ii)
  {
    dynamics_->eom(*next, actuation, &xd);
    Vector k1 = h*xd;
    dynamics_->eom(*next + k1/2, actuation, &xd);
    Vector k2 = h*xd;
    dynamics_->eom(*next + k2/2, actuation, &xd);
    Vector k3 = h*xd;
    dynamics_->eom(*next + k3, actuation, &xd);
    Vector k4 = h*xd;

    *next = *next + (k1+2*k2+2*k3+k4)/6;
  }
  
  return tau_;
}
