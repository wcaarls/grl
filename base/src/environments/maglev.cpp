/** \file maglev.cpp
 * \brief Maglev environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2024-04-17
 *
 * \copyright \verbatim
 * Copyright (c) 2024, Wouter Caarls
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

#include <grl/environments/maglev.h>

using namespace grl;

REGISTER_CONFIGURABLE(MagLevDynamics)
REGISTER_CONFIGURABLE(MagLevBalancingTask)

void MagLevDynamics::request(ConfigurationRequest *config)
{
  config->push_back(CRP("subbu", "Use Subbu's dynamics", subbu_, CRP::Configuration, 0, 1));
}

void MagLevDynamics::configure(Configuration &config)
{
  g_ = 9.81;
  M_ = 0.8;
  R_ = 11.68;
  x_inf_ = 0.007;
  L_inf_ = 0.8052;
  xi_ = 0.001599;
  
  subbu_ = config["subbu"];
}

void MagLevDynamics::reconfigure(const Configuration &config)
{
}

void MagLevDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  if (state.size() != 4 || actuation.size() != 1)
    throw Exception("dynamics/maglev requires a task/maglev subclass");

  xd->resize(4);
  (*xd)[3] = 1.;

  double d = state[0], dd = state[1], I = state[2], v = actuation[0];

  if (subbu_)
  {
    Eigen::Matrix3d J, R;
    Eigen::Vector3d gv;
    
    J  << 0, 1, 0, -1, 0, 0, 0, 0, 0;
    R  << 0, 0, 0, 0, 0, 0, 0, 0, R_;
    gv << 0, 0, 1;
    
    double dLq = xi_/pow(xi_+L_inf_*(x_inf_+d), 2);
    Eigen::Vector3d dH = VectorConstructor(-M_*g_+I*I*dLq/2, dd/M_, I*(x_inf_+d)/(xi_+L_inf_*(x_inf_+d)));
    
    Eigen::Vector3d r = (J-R)*dH+gv*v;
    (*xd)[0] = r[0];
    (*xd)[1] = r[1];
    (*xd)[2] = r[2];
  }
  else
  {
    double x = x_inf_+d, x2 = x*x;

    double alpha = g_-(xi_*I*I)/(2*M_*x2);
    double beta  = I*(xi_*dd-R_*x2)/(xi_*x+L_inf_*x2);
    double gamma = x/(xi_+L_inf_*x);
    
    (*xd)[0] = dd;
    (*xd)[1] = alpha;
    (*xd)[2] = beta + gamma*v;
  }
  
  // End stops
  if (state[0] <= 0. && (*xd)[0] < 0)
  {
    (*xd)[0] = 0;
    if ((*xd)[1] < 0)
      (*xd)[1] = 0;
  }
  else if (state[0] >= 0.013 && (*xd)[0] > 0)
  {
    (*xd)[0] = 0;
    if ((*xd)[1] > 0)
      (*xd)[1] = 0;
  }
}

// Balancing

void MagLevBalancingTask::request(ConfigurationRequest *config)
{
  Task::request(config);
}

void MagLevBalancingTask::configure(Configuration &config)
{
  config.set("observation_dims", 3);
  config.set("observation_min", VectorConstructor(0.   , -0.4, -5.));
  config.set("observation_max", VectorConstructor(0.013,  0.4,  5.));
  config.set("action_dims", 1);
  config.set("action_min", VectorConstructor(-60.));
  config.set("action_max", VectorConstructor( 60.));
  config.set("reward_min", -sqrt(0.0065));
  config.set("reward_max", 0.);
}

void MagLevBalancingTask::reconfigure(const Configuration &config)
{
}

void MagLevBalancingTask::start(int test, Vector *state)
{
  state->resize(4);
  (*state)[0] = 0.013;
  (*state)[1] = 0;
  (*state)[2] = 0;
  (*state)[3] = 0;
  
  // TODO: Warmup for Subbu
}

void MagLevBalancingTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  *reward = -pow(fabs(next[0]-0.0065), 0.5);
}

void MagLevBalancingTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  if (state.size() != 4)
    throw Exception("task/maglev/regulator requires dynamics/maglev");
    
  obs->v.resize(3);
  for (size_t ii=0; ii < 3; ++ii)
    (*obs)[ii] = state[ii];

  obs->absorbing = false;
  
  if (state[3] > .64)
    *terminal = 1;
  else
    *terminal = 0;
}

bool MagLevBalancingTask::invert(const Observation &obs, Vector *state, double time) const
{
  *state = extend(obs, VectorConstructor(time));
  
  return true;
}
