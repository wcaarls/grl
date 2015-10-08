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
 
#include <grl/environments/compass_walker/SWModel.h>
#include <grl/environments/compass_walker/compass_walker.h>

#define SLOPE_ANGLE 0.004

using namespace grl;

REGISTER_CONFIGURABLE(CompassWalkerModel)
REGISTER_CONFIGURABLE(CompassWalkerWalkTask)

// *** CompassWalkerModel ***

void CompassWalkerModel::request(ConfigurationRequest *config)
{
  config->push_back(CRP("control_step", "double.control_step", "Control step time", tau_, CRP::Configuration, 0.001, DBL_MAX));
  config->push_back(CRP("integration_steps", "Number of integration steps per control step", (int)steps_, CRP::Configuration, 1));
}

void CompassWalkerModel::configure(Configuration &config)
{
  tau_ = config["control_step"];
  steps_ = config["integration_steps"];

  model_.setSlopeAngle(SLOPE_ANGLE);
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
  if (state.size() != 8 || action.size() != 1)
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

  config->push_back(CRP("timeout", "Episode timeout", T_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("initial_state_variation", "Variation of initial state", initial_state_variation_, CRP::Configuration, 0., DBL_MAX));
}

void CompassWalkerWalkTask::configure(Configuration &config)
{
  T_ = config["timeout"];
  initial_state_variation_ = config["initial_state_variation"];

  config.set("observation_dims", 5);
  config.set("observation_min", VectorConstructor(-M_PI/8, -M_PI/4, -M_PI, -M_PI, 0.));
  config.set("observation_max", VectorConstructor( M_PI/8,  M_PI/4,  M_PI,  M_PI, 1.));
  config.set("action_dims", 1);
  config.set("action_min", VectorConstructor(-1.2));
  config.set("action_max", VectorConstructor( 1.2));
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
  while (swstate.getKinEnergy() + swstate.getHipY()*cos(SLOPE_ANGLE) < cos(SLOPE_ANGLE));

  state->resize(8);
  (*state)[CompassWalker::siStanceLegAngle] = swstate.mStanceLegAngle;
  (*state)[CompassWalker::siHipAngle] = swstate.mHipAngle;
  (*state)[CompassWalker::siStanceLegAngleRate] = swstate.mStanceLegAngleRate;
  (*state)[CompassWalker::siHipAngleRate] = swstate.mHipAngleRate;
  (*state)[CompassWalker::siStanceFootX] = swstate.mStanceFootX;
  (*state)[CompassWalker::siStanceLegChanged] = false;
  (*state)[CompassWalker::siLastHipX] = swstate.getHipX();
  (*state)[CompassWalker::siTime] = 0;
}

void CompassWalkerWalkTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 8)
    throw Exception("task/compass_walker/walk requires model/compass_walker");

  obs->resize(5);
  (*obs)[CompassWalker::siStanceLegAngle] = state[CompassWalker::siStanceLegAngle];
  (*obs)[CompassWalker::siHipAngle] = state[CompassWalker::siHipAngle] - 2 * state[CompassWalker::siStanceLegAngle];
  (*obs)[CompassWalker::siStanceLegAngleRate] = state[CompassWalker::siStanceLegAngleRate];
  (*obs)[CompassWalker::siHipAngleRate] = state[CompassWalker::siHipAngleRate] - 2 * state[CompassWalker::siStanceLegAngleRate];
  (*obs)[CompassWalker::siStanceLegChanged] = state[CompassWalker::siStanceLegChanged];
  
  if (fabs(state[CompassWalker::siStanceLegAngle]) > M_PI/8 || fabs(state[CompassWalker::siHipAngle] - 2 * state[CompassWalker::siStanceLegAngle]) > M_PI/4)
    *terminal = 2;
  else if (state[CompassWalker::siTime] > T_)
    *terminal = 1;
  else
    *terminal = 0;

  //obs->push_back(state[CompassWalker::siStanceFootX]); // TODO: Hack to record csv file. Remove asap.
}

void CompassWalkerWalkTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != 8 || action.size() != 1 || next.size() != 8)
    throw Exception("task/compass_walker/walk requires model/compass_walker");

  timestep_cnt_++;

  *reward = -1*0;

  // Instead of using LastHipX, which is non-Markov, assume the last step
  // was just as long as this one.
  if (next[CompassWalker::siStanceLegChanged])
  {
    //*reward = fmin(50 * 4 * sin(next[CompassWalker::siStanceLegAngle]), 30);
    double velocity = sin(next[CompassWalker::siStanceLegAngle]) / (timestep_cnt_*0.2);
    *reward = 30*exp(-(pow(velocity - 0.122, 2.0))/0.0053);
    timestep_cnt_ = 0;
  }

  if (fabs(next[CompassWalker::siStanceLegAngle]) > M_PI/8 || fabs(next[CompassWalker::siHipAngle] - 2 * next[CompassWalker::siStanceLegAngle]) > M_PI/4)
    *reward = -100;
}

bool CompassWalkerWalkTask::invert(const Vector &obs, Vector *state) const
{
  state->resize(8);

  (*state)[CompassWalker::siStanceLegAngle] = obs[CompassWalker::siStanceLegAngle];
  (*state)[CompassWalker::siHipAngle] = obs[CompassWalker::siHipAngle] + 2 * obs[CompassWalker::siStanceLegAngle];
  (*state)[CompassWalker::siStanceLegAngleRate] = obs[CompassWalker::siStanceLegAngleRate];
  (*state)[CompassWalker::siHipAngleRate] = obs[CompassWalker::siHipAngleRate] + 2 * obs[CompassWalker::siStanceLegAngleRate];
  (*state)[CompassWalker::siStanceLegChanged] = obs[CompassWalker::siStanceLegChanged];
  (*state)[CompassWalker::siStanceFootX] = 0;
  (*state)[CompassWalker::siLastHipX] = 0;
  (*state)[CompassWalker::siTime] = 0;

  return true;
}
