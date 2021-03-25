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

// *** TennesseeEastmanTask

void TennesseeEastmanTask::request(ConfigurationRequest *config)
{
  Task::request(config);

  config->push_back(CRP("timeout", "Episode timeout", T_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("randomization", "Level of start state randomization", randomization_, CRP::Configuration, 0., 1.));

  config->push_back(CRP("control_step", "double.control_step", "Control step time", tau_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("action_step", "double.action_step", "RL action step time", action_tau_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("control", "What kind of variables the RL policy controls", control_, CRP::Configuration, {"setpoint", "action"}));
  config->push_back(CRP("observation_idx", "Which variables (XMEAS) the RL policy observes", observation_idx_, CRP::Configuration));
  config->push_back(CRP("action_idx", "Which variables (SETPT or XMV) the RL policy controls", action_idx_, CRP::Configuration));
  config->push_back(CRP("terminal_penalty", "Penalty applied when simulation terminates prematurely", terminal_penalty_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("time_reward", "Base reward given per timestep", time_reward_, CRP::Configuration, 0., DBL_MAX));
}

void TennesseeEastmanTask::configure(Configuration &config)
{
  T_ = config["timeout"];
  randomization_ = config["randomization"];
  tau_ = config["control_step"];
  action_tau_ = config["action_step"];
  control_ = config["control"].str();
  observation_idx_ = config["observation_idx"].v();
  action_idx_ = config["action_idx"].v();
  terminal_penalty_ = config["terminal_penalty"];
  time_reward_ = config["time_reward"];
  
  if (!action_tau_)
    action_tau_ = tau_;
    
  if (!observation_idx_.size())
  {
    observation_idx_.resize(41);
    for (size_t ii=0; ii != observation_idx_.size(); ++ii)
      observation_idx_[ii] = ii;
  }

  if (!action_idx_.size())
  {
    action_idx_.resize(control_[0]=='s' ?20:12);
    for (size_t ii=0; ii != action_idx_.size(); ++ii)
      action_idx_[ii] = ii;
  }
  
  for (size_t ii=0; ii != action_idx_.size(); ++ii)  
    if (action_idx_[ii] != (int)action_idx_[ii] || action_idx_[ii] < 0 || (control_[0] == 's' && action_idx_[ii] >= 20) || (control_[0] == 'a' && action_idx_[ii] >= 12))
      throw bad_param("mapping/policy/tennessee:action_idx");
      
  double xmeas_min[] = {
       0, 3000,  100,    5,   15,   25, 2500,    0,   30,    0,
      30,    0, 2500,   10,    0, 2500,   15,   30,    0,  200,
      30,   30,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0
  }, xmeas_max[] = {
     1.5, 6000, 3000,   10,   25,   40, 3000,  100,  175,  1.5,
     175,  100, 3000,   25,  100, 3500,   25,  175,   15,  400,
     175,  175,  100,  100,  100,  100,  100,  100,  100,  100,
     100,  100,  100,  100,  100,  100,  100,  100,  100,  100,
     100
  };

  size_t setpt_idx[] = {
     1,  2,  0,  3,  4,  9, 11, 14, 18, 20,
    16, 12, 22, 25, 26, 17,  7,  8, 29, 37};
         
  Vector observation_min(observation_idx_.size()), observation_max(observation_idx_.size()), action_min(action_idx_.size()), action_max(action_idx_.size());
  
  for (size_t ii=0; ii != observation_idx_.size(); ++ii)
  {
    observation_min[ii] = xmeas_min[(size_t)observation_idx_[ii]];
    observation_max[ii] = xmeas_max[(size_t)observation_idx_[ii]];
  }

  for (size_t ii=0; ii != action_idx_.size(); ++ii)
  {
    if (control_[0] == 's')
    {
      action_min[ii] = xmeas_min[setpt_idx[(size_t)action_idx_[ii]]];
      action_max[ii] = xmeas_max[setpt_idx[(size_t)action_idx_[ii]]];
    }
    else
    {
      action_min[ii] = 0;
      action_max[ii] = 100;
    }
  }
  
  config.set("observation_dims", observation_idx_.size());
  config.set("observation_min", observation_min);
  config.set("observation_max", observation_max);
  config.set("action_dims", action_idx_.size());
  config.set("action_min", action_min);
  config.set("action_max", action_max);
  config.set("reward_min", 0.);
  config.set("reward_max", 1.);
}

void TennesseeEastmanTask::reconfigure(const Configuration &config)
{
}

void TennesseeEastmanTask::start(int test, Vector *state)
{
  state->resize(51);
  
  int NN=50;
  double TIME=0, YP[50];
  teinit_(&NN, &TIME, state->data(), YP);

  ctrlall_.DELTAT = tau_ / 3600.;
  teinct_();

  double state_min[] = {
    8.324 ,3.491 ,6.056 ,0.338 ,19.324 ,2.354 ,123.502 ,127.349 ,2.247 ,51.005 ,21.392 ,37.108 ,0.197 ,12.164 ,1.482 ,41.957 ,32.963 ,0.456 ,0.344 ,0.006 ,0.724 ,0.013 ,0.601 ,0.071 ,38.622 ,31.508 ,0.300 ,86.205 ,23.818 ,70.660 ,18.431 ,50.287 ,4.437 ,9.538 ,4.444 ,0.737 ,75.679 ,61.838 ,50.442 ,43.184 ,19.715 ,49.042 ,17.768 ,32.051 ,30.480 ,37.227 ,37.957 ,32.885 ,14.491 ,40.000
  };

  double state_max[] = {
    12.486 ,5.237 ,9.084 ,0.508 ,28.986 ,3.531 ,185.252 ,191.024 ,3.370 ,76.507 ,32.088 ,55.662 ,0.296 ,18.246 ,2.223 ,62.936 ,49.445 ,0.684 ,0.517 ,0.010 ,1.087 ,0.019 ,0.901 ,0.106 ,57.933 ,47.262 ,0.451 ,129.308 ,35.727 ,105.990 ,27.647 ,75.430 ,6.656 ,14.307 ,6.667 ,1.106 ,113.519 ,92.756 ,75.663 ,64.776 ,29.572 ,73.562 ,26.652 ,48.076 ,45.720 ,55.841 ,56.935 ,49.327 ,21.736 ,60.000
  };

  for (int ii=0; ii < 50; ii++)
    (*state)[ii] = std::min(std::max((*state)[ii] + RandGen::get()*(test == 0)*randomization_*(state_max[ii]-state_min[ii]), state_min[ii]), state_max[ii]);
  (*state)[50] = 0;
}

bool TennesseeEastmanTask::actuate(const Vector &prev, const Vector &state, const Action &action, Vector *actuation) const
{
  if (prev.size() != 51 || action.size() != action_idx_.size() || state.size() != 51)
  {
    ERROR("Expected prev/action/state size 51/" << action_idx_.size() << "/51, got " << prev.size() << "/" << action.size() << "/" << state.size()); 
    throw Exception("task/tennessee requires dynamics/tennessee");
  }
  
  // Avoid updating ERROLD on last actuation
  if ((state[50] - prev[50]) > action_tau_-1e-7)
    return true;

  if (control_[0] == 's')
  {
    // Copy action into manipulated setpoints 
    for (size_t ii=0; ii != action_idx_.size(); ++ii)
      ctrlall_.SETPT[(size_t)action_idx_[ii]] = action[ii];
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
    for (size_t ii=0; ii != action_idx_.size(); ++ii)
      (*actuation)[(size_t)action_idx_[ii]] = action[ii];
  }
  
  return false;
}

void TennesseeEastmanTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != 51)
  {
    ERROR("Expected state size 51, got " << state.size());
    throw Exception("task/tennessee requires dynamics/tennessee");
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
  
  obs->v.resize(observation_idx_.size());
  for (size_t ii=0; ii != observation_idx_.size(); ++ii)
    (*obs)[ii] = pv_.XMEAS[(size_t)observation_idx_[ii]];

  if (*terminal == 2)
    obs->absorbing = true;
  else
    obs->absorbing = false;
}

void TennesseeEastmanTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  if (state.size() != 51 || action.size() != action_idx_.size() || next.size() != 51)
  {
    ERROR("Expected state/action/next size 51/" << action_idx_.size() << "/51, got " << state.size() << "/" << action.size() << "/" << next.size()); 
    throw Exception("task/tennessee requires dynamics/tennessee");
  }

  // Update XMEAS
  int NN=50;
  double YP[50], TIME=next[50]/3600.;
  tefunc_(&NN, &TIME, next.data(), YP);

  // See if eom bailed  
  bool terminal = true;
  for (size_t ii=0; ii != 50; ++ii)
    if (YP[ii] != 0)
    {
      terminal = false;
      break;
    }

  if (!terminal)
    *reward = calculateReward(pv_.XMEAS, action);
  else
    *reward = -terminal_penalty_;
  
  // Normalize reward per timestep.
  *reward *= (next[50]-state[50]);
}

double TennesseeEastmanRegulationTask::calculateReward(const double *XMEAS, const Action &action) const
{
  Eigen::VectorXd gan1 = VectorConstructor(0.0318, 0.0536);
  Eigen::VectorXd gan2 = VectorConstructor(2.209, 6.177, 22.06, 14.56, 17.89, 30.44, 22.94);
  Eigen::VectorXd gan3 = VectorConstructor(0.2206, 0.1456, 0.1789);
  
  Eigen::Map<const Eigen::VectorXd > XMEASv(XMEAS, 41);
  
  double c1 = XMEASv.segment(18,2).dot(gan1);
  double c2 = XMEASv.segment(28,7).dot(gan2);
  double c3 = XMEASv.segment(36,3).dot(gan3);
  
  double cost = (c1 + c2*XMEAS[9]*0.44791 + c3*action[7]*4.451)/3600;
  
  return time_reward_ - cost;
}
