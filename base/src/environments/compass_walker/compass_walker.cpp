/** \file compass_walker.cpp
 * \brief Compass walker environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-03-14
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

#include <cmath>
#include <grl/environments/compass_walker/SWModel.h>
#include <grl/environments/compass_walker/compass_walker.h>

using namespace grl;

REGISTER_CONFIGURABLE(CompassWalkerModel)
REGISTER_CONFIGURABLE(CompassWalkerWalkTask)
REGISTER_CONFIGURABLE(CompassWalkerVrefTask)
REGISTER_CONFIGURABLE(CompassWalkerVrefuTask)
// *** CompassWalkerModel ***

void CompassWalkerModel::request(ConfigurationRequest *config)
{
  config->push_back(CRP("control_step", "double.control_step", "Control step time", tau_, CRP::Configuration, 0.001, DBL_MAX));
  config->push_back(CRP("integration_steps", "Number of integration steps per control step", (int)steps_, CRP::Configuration, 1));
  config->push_back(CRP("slope_angle", "double.slope_angle", "Inclination of the slope", slope_angle_, CRP::Configuration, -DBL_MAX, DBL_MAX));
}

void CompassWalkerModel::configure(Configuration &config)
{
  tau_ = config["control_step"];
  steps_ = config["integration_steps"];
  slope_angle_ = config["slope_angle"];

  model_.setSlopeAngle(slope_angle_);
  model_.setTiming(tau_, steps_);
}

void CompassWalkerModel::reconfigure(const Configuration &config)
{
}

CompassWalkerModel *CompassWalkerModel::clone() const
{
  return new CompassWalkerModel(*this);
}

double CompassWalkerModel::step(const Vector &state, const Vector &action, Vector *next) const
{
  if (state.size() != CompassWalker::ssStateSize || action.size() != 1)
    throw Exception("model/compass_walker requires a task/compass_walker subclass");

  CSWModelState swstate;
  
  swstate.init(state[CompassWalker::siStanceFootX],
               state[CompassWalker::siStanceLegAngle],
               state[CompassWalker::siStanceLegAngleRate],
               state[CompassWalker::siHipAngle],
               state[CompassWalker::siHipAngleRate]);

  model_.singleStep(swstate, action[0]);

  next->resize(state.size());
  (*next)[CompassWalker::siStanceLegAngle] = swstate.mStanceLegAngle;
  (*next)[CompassWalker::siHipAngle] = swstate.mHipAngle;
  (*next)[CompassWalker::siStanceLegAngleRate] = swstate.mStanceLegAngleRate;
  (*next)[CompassWalker::siHipAngleRate] = swstate.mHipAngleRate;
  (*next)[CompassWalker::siStanceFootX] = swstate.mStanceFootX;
  (*next)[CompassWalker::siStanceLegChanged] = swstate.mStanceLegChanged;
  if (swstate.mStanceLegChanged)
    (*next)[CompassWalker::siLastHipX] = swstate.getHipX();
  else
    (*next)[CompassWalker::siLastHipX] = state[CompassWalker::siLastHipX];
  (*next)[CompassWalker::siTime] = state[CompassWalker::siTime] + tau_;

  return tau_;
}

// *** CompassWalkerWalkTask ***

void CompassWalkerWalkTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("timeout", "Learning episode timeout", T_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("initial_state_variation", "Variation of initial state", initial_state_variation_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("slope_angle", "double.slope_angle", "Inclination of the slope", slope_angle_, CRP::System, -DBL_MAX, DBL_MAX));
  config->push_back(CRP("negative_reward", "Negative reward", neg_reward_, CRP::Configuration, -DBL_MAX, 0.));
  config->push_back(CRP("observe", "Negative reward", observe_, CRP::Configuration));
}

void CompassWalkerWalkTask::configure(Configuration &config)
{
  T_ = config["timeout"];
  initial_state_variation_ = config["initial_state_variation"];
  slope_angle_ = config["slope_angle"];
  neg_reward_ = config["negative_reward"];

  observe_ = config["observe"];

  if (observe_.size() != CompassWalker::osMaxObservationSize)
    throw bad_param("task/walk:observe");
  obs_.resize(observe_.size());

  observation_dims_ = (observe_.array() != 0).count();

  config.set("observation_dims", observation_dims_);
  config.set("observation_min", VectorConstructor(-M_PI/8, -M_PI/4, -M_PI, -M_PI, 0));
  config.set("observation_max", VectorConstructor( M_PI/8,  M_PI/4,  M_PI,  M_PI, 0.5));
  config.set("action_dims", 1);
  config.set("action_min", VectorConstructor(-10));
  config.set("action_max", VectorConstructor( 10));
  config.set("reward_min", -101);
  config.set("reward_max",  50);
}

void CompassWalkerWalkTask::reconfigure(const Configuration &config)
{
}

CompassWalkerWalkTask *CompassWalkerWalkTask::clone() const
{
  return new CompassWalkerWalkTask(*this);
}

void CompassWalkerWalkTask::start(int test, Vector *state) const
{
  CSWModelState swstate, initial_state;

  initial_state.init(0, 0.1534, -0.1561, 2.0*0.1534, -0.0073);

  swstate.mStanceFootX = 0;

  do
  {
    swstate.mStanceLegAngle     = initial_state.mStanceLegAngle     * (1.0-(initial_state_variation_) + 2.0*initial_state_variation_*drand48());
    swstate.mHipAngle           = initial_state.mHipAngle           * (1.0-(initial_state_variation_) + 2.0*initial_state_variation_*drand48());
    swstate.mStanceLegAngleRate = initial_state.mStanceLegAngleRate * (1.0-(initial_state_variation_) + 2.0*initial_state_variation_*drand48());
    swstate.mHipAngleRate       = initial_state.mHipAngleRate       * (1.0-(initial_state_variation_) + 2.0*initial_state_variation_*drand48());
  }
  while (swstate.getKinEnergy() + swstate.getHipY()*cos(slope_angle_) < cos(slope_angle_));

  state->resize(CompassWalker::ssStateSize);
  (*state)[CompassWalker::siStanceLegAngle] = swstate.mStanceLegAngle;
  (*state)[CompassWalker::siHipAngle] = swstate.mHipAngle;
  (*state)[CompassWalker::siStanceLegAngleRate] = swstate.mStanceLegAngleRate;
  (*state)[CompassWalker::siHipAngleRate] = swstate.mHipAngleRate;
  (*state)[CompassWalker::siStanceFootX] = swstate.mStanceFootX;
  (*state)[CompassWalker::siStanceLegChanged] = false;
  (*state)[CompassWalker::siLastHipX] = swstate.getHipX();
  (*state)[CompassWalker::siTime] = 0;

  if (test)
    timeout_ = 2*T_;
  else
    timeout_ = T_;
}

void CompassWalkerWalkTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != CompassWalker::ssStateSize)
    throw Exception("task/compass_walker/walk requires model/compass_walker");

  obs_[CompassWalker::oiStanceLegAngle] = state[CompassWalker::siStanceLegAngle];
  obs_[CompassWalker::oiHipAngle] = state[CompassWalker::siHipAngle] - 2 * state[CompassWalker::siStanceLegAngle];
  obs_[CompassWalker::oiStanceLegAngleRate] = state[CompassWalker::siStanceLegAngleRate];
  obs_[CompassWalker::oiHipAngleRate] = state[CompassWalker::siHipAngleRate] - 2 * state[CompassWalker::siStanceLegAngleRate];
  obs_[CompassWalker::oiStanceLegChanged] = state[CompassWalker::siStanceLegChanged] > 0.5;

  // Calculate average velocity
  double velocity = -state[CompassWalker::siStanceLegAngleRate] * cos(state[CompassWalker::siStanceLegAngle]);
  hip_instant_velocity_.push_back(velocity);
  if (hip_instant_velocity_.size() >= 100)
    hip_instant_velocity_.pop_front();
  double sum = std::accumulate(hip_instant_velocity_.begin(), hip_instant_velocity_.end(), 0.0);
  hip_avg_velocity_ = sum / hip_instant_velocity_.size();
  obs_[CompassWalker::oiHipAvgVelocity] = hip_avg_velocity_;

  // Mask unwanted observations
  obs->resize(observation_dims_);
  for (int i = 0, j = 0; i < observe_.size(); i++)
    if (observe_[i] != 0)
      (*obs)[j++] = obs_[i];

  if (fabs(state[CompassWalker::siStanceLegAngle]) > M_PI/8 || fabs(state[CompassWalker::siHipAngle] - 2 * state[CompassWalker::siStanceLegAngle]) > M_PI/4)
    *terminal = 2;
  else if (state[CompassWalker::siTime] > timeout_)
    *terminal = 1;
  else
    *terminal = 0;

  if ((*terminal) && verbose_)
    std::cout << hip_avg_velocity_ << std::endl;
}

void CompassWalkerWalkTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != CompassWalker::ssStateSize || action.size() != 1 || next.size() != CompassWalker::ssStateSize)
    throw Exception("task/compass_walker/walk requires model/compass_walker");

  *reward = -1;

  // Instead of using LastHipX, which is non-Markov, assume the last step
  // was just as long as this one.
  if (next[CompassWalker::siStanceLegChanged] > 0.5)
    *reward = fmin(50 * 4 * sin(next[CompassWalker::siStanceLegAngle]), 30);

  if (fabs(next[CompassWalker::siStanceLegAngle]) > M_PI/8 || fabs(next[CompassWalker::siHipAngle] - 2 * next[CompassWalker::siStanceLegAngle]) > M_PI/4)
    if (neg_reward_)
      *reward = neg_reward_;
}

bool CompassWalkerWalkTask::invert(const Vector &obs, Vector *state) const
{
  state->resize(CompassWalker::ssStateSize);

  (*state)[CompassWalker::siStanceLegAngle] = obs[CompassWalker::siStanceLegAngle];
  (*state)[CompassWalker::siHipAngle] = obs[CompassWalker::siHipAngle] + 2 * obs[CompassWalker::siStanceLegAngle];
  (*state)[CompassWalker::siStanceLegAngleRate] = obs[CompassWalker::siStanceLegAngleRate];
  (*state)[CompassWalker::siHipAngleRate] = obs[CompassWalker::siHipAngleRate] + 2 * obs[CompassWalker::siStanceLegAngleRate];
  (*state)[CompassWalker::siStanceLegChanged] = obs[CompassWalker::siStanceLegChanged] > 0.5;
  (*state)[CompassWalker::siStanceFootX] = 0;
  (*state)[CompassWalker::siLastHipX] = 0;
  (*state)[CompassWalker::siTime] = 0;

  return true;
}

// *** CompassWalkerVrefTask ***
void CompassWalkerVrefTask::start(int test, Vector *state) const
{
  CompassWalkerWalkTask::start(test, state);
  hip_avg_velocity_ = 0.05;
  hip_instant_velocity_.clear();
}

void CompassWalkerVrefTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != CompassWalker::ssStateSize || action.size() != 1 || next.size() != CompassWalker::ssStateSize)
    throw Exception("task/compass_walker/vref requires model/compass_walker");

  *reward = 0.1*fmax(0, 4 - 100.0*pow(hip_avg_velocity_ - vref_, 2));

  if (fabs(next[CompassWalker::siStanceLegAngle]) > M_PI/8 || fabs(next[CompassWalker::siHipAngle] - 2 * next[CompassWalker::siStanceLegAngle]) > M_PI/4)
    if (neg_reward_)
      *reward = neg_reward_;
}

void CompassWalkerVrefTask::request(ConfigurationRequest *config)
{
  Task::request(config);
  CompassWalkerWalkTask::request(config);
  config->push_back(CRP("reference_velocity", "Reference velocity", vref_, CRP::Configuration, 0., DBL_MAX));
}

void CompassWalkerVrefTask::configure(Configuration &config)
{
  CompassWalkerWalkTask::configure(config);
  vref_ = config["reference_velocity"];
}

// *** CompassWalkerVrefuTask ***
void CompassWalkerVrefuTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != CompassWalker::ssStateSize || action.size() != 1 || next.size() != CompassWalker::ssStateSize)
    throw Exception("task/compass_walker/vrefu requires model/compass_walker");

  CompassWalkerVrefTask::evaluate(state, action, next, reward);

  *reward -= 0.01*action[0]*action[0]; // add action penalty to the reward

  if (fabs(next[CompassWalker::siStanceLegAngle]) > M_PI/8 || fabs(next[CompassWalker::siHipAngle] - 2 * next[CompassWalker::siStanceLegAngle]) > M_PI/4)
    if (neg_reward_)
      *reward = neg_reward_;
}
