/** \file pendulum.cpp
 * \brief Pendulum environment source file.
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

#include <grl/environments/pendulum.h>

using namespace grl;

REGISTER_CONFIGURABLE(PendulumDynamics)
REGISTER_CONFIGURABLE(PendulumSwingupTask)

void PendulumDynamics::request(ConfigurationRequest *config)
{
}

void PendulumDynamics::configure(Configuration &config)
{
  J_ = 0.000191;
  m_ = 0.055;
  g_ = 9.81;
  l_ = 0.042;
  b_ = 0.000003;
  K_ = 0.0536;
  R_ = 9.5;
}

void PendulumDynamics::reconfigure(const Configuration &config)
{
}

PendulumDynamics *PendulumDynamics::clone() const
{
  return new PendulumDynamics(*this);
}

void PendulumDynamics::eom(const Vector &state, const Vector &action, Vector *xd) const
{
  if (state.size() != 3 || action.size() != 1)
    throw Exception("dynamics/pendulum requires a task/pendulum subclass");

  double a   = state[0];
  double ad  = state[1];
  double add = (1/J_)*(m_*g_*l_*sin(a)-b_*ad-(K_*K_/R_)*ad+(K_/R_)*action[0]);
  
  xd->resize(3);
  (*xd)[0] = ad;
  (*xd)[1] = add;
  (*xd)[2] = 1;
}

void PendulumSwingupTask::request(ConfigurationRequest *config)
{
  config->push_back(CRP("timeout", "Episode timeout", T_, CRP::Configuration, 0.));
}

void PendulumSwingupTask::configure(Configuration &config)
{
  T_ = config["timeout"];

  config.set("observation_min", VectorConstructor(0., -12*M_PI));
  config.set("observation_max", VectorConstructor(2*M_PI, 12*M_PI));
  config.set("action_min", VectorConstructor(-3));
  config.set("action_max", VectorConstructor(3));
}

void PendulumSwingupTask::reconfigure(const Configuration &config)
{
}

PendulumSwingupTask *PendulumSwingupTask::clone() const
{
  return new PendulumSwingupTask(*this);
}

void PendulumSwingupTask::start(Vector *state) const
{
  state->resize(3);
  (*state)[0] = M_PI;
  (*state)[1] = 0;
  (*state)[2] = 0;
}

void PendulumSwingupTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 3)
    throw Exception("task/pendulum/swingup requires dynamics/pendulum");

  double a = fmod(state[0]+M_PI, 2*M_PI);
  if (a < 0) a += 2*M_PI;
  
  obs->resize(2);
  (*obs)[0] = a;
  (*obs)[1] = state[1];
  if (state[2] > T_)
    *terminal = 1;
  else
    *terminal = 0;
}

bool PendulumSwingupTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != 3 || action.size() != 1 || next.size() != 3)
    throw Exception("task/pendulum/swingup requires dynamics/pendulum");

  double a = fmod(fabs(next[0]), 2*M_PI);
  if (a > M_PI) a -= 2*M_PI;

  *reward = -5*pow(a, 2) - 0.1*pow(next[1], 2) - 1*pow(action[0], 2);
  
  return true;
}
