/** \file te.cpp
 * \brief Tennessee Eastman process environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-05-17
 *
 * \copyright \verbatim
 * Copyright (c) 2020, Wouter Caarls
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

#include <grl/environments/tennessee.h>

using namespace grl;

REGISTER_CONFIGURABLE(TennesseeEastmanDynamics)
REGISTER_CONFIGURABLE(TennesseeEastmanRegulationTask)

extern "C" {

extern struct
{
  double XMEAS[41];
  double XMV[12];
} pv_;

extern struct
{
  double SETPT[20];
  double DELTAT;
} ctrlall_; 

void teinit_(const int *NN, const double *TIME, const double *YY, double *YP);
void tefunc_(const int *NN, const double *TIME, const double *YY, double *YP);

void teinct_();
void tecntr_();

} // extern "C"

// *** TennesseeEastmanDynamics

void TennesseeEastmanDynamics::request(ConfigurationRequest *config)
{
}

void TennesseeEastmanDynamics::configure(Configuration &config)
{
}

void TennesseeEastmanDynamics::reconfigure(const Configuration &config)
{
}

void TennesseeEastmanDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  if (state.size() != 51 || actuation.size() != 12)
  {
    ERROR("Expected state/actuation size 51/12, got " << state.size() << "/" << actuation.size()); 
    throw Exception("dynamics/tennessee requires a task/tennessee subclass");
  }

  for (size_t ii=0; ii != 12; ++ii)
    pv_.XMV[ii] = actuation[ii];

  xd->resize(51);
  
  int NN=50;
  double TIME = state[50]/3600.;
  tefunc_(&NN, &TIME, state.data(), xd->data());
  
  for (size_t ii=0; ii != 50; ++ii)
    (*xd)[ii] /= 3600.;

  (*xd)[50] = 1;
}

// *** TennesseeEastmanRegulationTask

void TennesseeEastmanRegulationTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("timeout", "Episode timeout", T_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("randomization", "Level of start state randomization", randomization_, CRP::Configuration, 0., 1.));

  config->push_back(CRP("control_step", "double.control_step", "Control step time", tau_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("action_step", "double.action_step", "RL action step time", action_tau_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("control", "What kind of variables the RL policy controls", control_, CRP::Configuration, {"setpoint", "action"}));
  config->push_back(CRP("variables", "Which variables the RL policy controls", variables_, CRP::Configuration));
}

void TennesseeEastmanRegulationTask::configure(Configuration &config)
{
  T_ = config["timeout"];
  randomization_ = config["randomization"];
  tau_ = config["control_step"];
  action_tau_ = config["action_step"];
  control_ = config["control"].str();
  variables_ = config["variables"].v();
  
  if (!action_tau_)
    action_tau_ = tau_;
  
  for (size_t ii=0; ii != variables_.size(); ++ii)  
    if (variables_[ii] != (int)variables_[ii] || variables_[ii] < 0 || variables_[ii] >= 12)
      throw bad_param("mapping/policy/tennessee:variables");

  config.set("observation_dims", 41);
  config.set("observation_min", ConstantVector(41, 0.));
  config.set("observation_max", ConstantVector(41, 1.));
  config.set("action_dims", 12);
  config.set("action_min", ConstantVector(12, 0.));
  config.set("action_max", ConstantVector(12, 1.));
  config.set("reward_min", 0.);
  config.set("reward_max", 1.);
}

void TennesseeEastmanRegulationTask::reconfigure(const Configuration &config)
{
}

void TennesseeEastmanRegulationTask::start(int test, Vector *state)
{
  state->resize(51);
  
  int NN=50;
  double TIME=0, YP[50];
  teinit_(&NN, &TIME, state->data(), YP);

  ctrlall_.DELTAT = tau_ / 3600.;
  teinct_();

  // TODO: Do some randomization on initial state
    
  (*state)[50] = 0;
}

bool TennesseeEastmanRegulationTask::actuate(const Vector &prev, const Vector &state, const Action &action, Vector *actuation) const
{
  if (prev.size() != 51 || action.size() != variables_.size() || state.size() != 51)
  {
    ERROR("Expected prev/action/state size 51/" << variables_.size() << "/51, got " << prev.size() << "/" << action.size() << "/" << state.size()); 
    throw Exception("task/tennessee/regulation requires dynamics/tennessee");
  }
  
  // Avoid updating ERROLD on last actuation
  if ((state[50] - prev[50]) > action_tau_-1e-7)
    return true;

  if (control_[0] == 's')
  {
    // Copy action into manipulated setpoints 
    for (size_t ii=0; ii != variables_.size(); ++ii)
      ctrlall_.SETPT[(size_t)variables_[ii]] = action[ii];
  }
  
  // Run baseline policy
  int NN=50;
  double TIME = state[50]/3600., YP[50];
  tefunc_(&NN, &TIME, state.data(), YP);

  // WARNING: INTERNAL STATE
  tecntr_();
  
  actuation->resize(12);
  for (size_t ii=0; ii != 12; ++ii)
    (*actuation)[ii] = pv_.XMV[ii];
    
  if (control_[0] == 'a')
  {
    // Copy action into manipulated actuation
    for (size_t ii=0; ii != variables_.size(); ++ii)
      (*actuation)[(size_t)variables_[ii]] = action[ii];
  }
  
  return false;
}

void TennesseeEastmanRegulationTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != 51)
  {
    ERROR("Expected state size 51, got " << state.size());
    throw Exception("task/tennessee/regulation requires dynamics/tennessee");
  }
    
  int NN=50;
  double TIME=state[50]/3600.,YP[50];
  tefunc_(&NN, &TIME, state.data(), YP);
  
  *terminal = 2;
  for (size_t ii=0; ii != 50; ++ii)
    if (YP[ii] != 0)
    {
      *terminal = 0;
      break;
    }

  if (!*terminal && state[50] > T_)
    *terminal = 1;
  
  obs->v.resize(41);
  for (size_t ii=0; ii != obs->size(); ++ii)
    (*obs)[ii] = pv_.XMEAS[ii];

  if (*terminal == 2)
    obs->absorbing = true;
  else
    obs->absorbing = false;
}

void TennesseeEastmanRegulationTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  if (state.size() != 51 || action.size() != variables_.size() || next.size() != 51)
  {
    ERROR("Expected state/action/next size 51/" << variables_.size() << "/51, got " << state.size() << "/" << action.size() << "/" << next.size()); 
    throw Exception("task/tennessee/regulation requires dynamics/tennessee");
  }

  int NN=50;
  double YP[50], TIME=state[50]/3600.;
  tefunc_(&NN, &TIME, state.data(), YP);
  
  double XMEAS1[41], *XMEAS2 = pv_.XMEAS;
  for (size_t ii=0; ii != 41; ++ii)
    XMEAS1[ii] = pv_.XMEAS[ii];

  TIME = next[50]/3600.;
  tefunc_(&NN, &TIME, next.data(), YP);
  
  // TODO: Calculate reward using XMEAS1, XMEAS2 and action
  *reward = 0;
  
  // Normalize reward per timestep.
  *reward *= (next[50]-state[50]);
}
