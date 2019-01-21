/** \file mountain.cpp
 * \brief Mountain world environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-10
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
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

#include <grl/environments/mountain.h>

using namespace grl;

REGISTER_CONFIGURABLE(MountainDynamics)
REGISTER_CONFIGURABLE(MountainRegulatorTask)

void MountainDynamics::request(ConfigurationRequest *config)
{
  config->push_back(CRP("mass", "Car mass", m_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("gravity", "Gravitational acceleration", g_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("friction", "Coefficient of viscous friction between car and ground", mu_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("stiffness", "Spring constant of walls", k_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("map", "mapping", "Height map", map_));
}

void MountainDynamics::configure(Configuration &config)
{
  map_ = (Mapping*)config["map"].ptr();
  m_ = config["mass"];
  g_ = config["gravity"];
  mu_ = config["friction"];
  k_ = config["stiffness"];
}

void MountainDynamics::reconfigure(const Configuration &config)
{
}

void MountainDynamics::slope(const Vector &pos, Vector *angle) const
{
  Vector h;
  angle->resize(2);
  
  for (size_t ii=0; ii < 2; ++ii)
  {
    Vector posd = pos;
    posd[ii] -= 0.01;
    double height1 = map_->read(posd, &h);
    
    posd[ii] += 0.02;
    double height2 = map_->read(posd, &h);
    
    posd[ii] -= 0.01;
    (*angle)[ii] = atan2(height2-height1, 0.02);
  }
}

void MountainDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  if (state.size() != 5 || actuation.size() != 2)
  {
    ERROR("Received state/actuation size " << state.size() << "/" << actuation.size() << ", expected 5/2");
    throw Exception("dynamics/mountain requires a task/mountain subclass");
  }

  Vector d, pos, angle;
  
  slope(state.head(2), &angle);
  
  xd->resize(5);
  
  // Velocity/acceleration is defined along ground
  (*xd)[0] = state[2];
  (*xd)[1] = state[3];
  (*xd)[4] = 1;

  double xdot = state[2], ydot = state[3], ux = actuation[0], uy = actuation[1];
  double ax = angle[0], ay = angle[1];
  double a;
  
/*
  double Fg = m_*g_, Fp, Fn, Ff, Fr;
  double vx = xdot/cos(ax), vy = ydot/cos(ay);
  Fp = Fg*sin(ax);
  Fn = Fg*cos(ax);
  Ff = mu_*Fn*vx;
  Fr = ux-Ff-Fp;
  a  = Fr/m_;
*/  

  a = ux/m_-g_*(mu_*xdot+sin(ax));
  (*xd)[2] = a*cos(ax);

/*
  Fp = Fg*sin(ay);
  Fn = Fg*cos(ay);
  Ff = mu_*Fn*vy;
  Fr = uy-Ff-Fp;
  a  = Fr/m_;
*/
  a = uy/m_-g_*(mu_*ydot+sin(ay));
  (*xd)[3] = a*cos(ay);
  
//  NOTICE(state.head(2) << " -> " << angle << " -> [" << -g_*(sin(ax)) << ", " << -g_*(sin(ay)) << "]");
  
  // Don't fall off the world
  if (state[0] < 0)
    (*xd)[2] -= k_ * state[0];
  if (state[0] > 1)
    (*xd)[2] -= k_ * (state[0]-1);
  if (state[1] < 0)
    (*xd)[3] -= k_ * state[1];
  if (state[1] > 1)
    (*xd)[3] -= k_ * (state[1]-1);
}

void MountainRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  if (q_.size() != 4)
    throw bad_param("task/mountain/regulator:q");
  if (r_.size() != 2)
    throw bad_param("task/mountain/regulator:r");

  config.set("observation_min", VectorConstructor(0., 0., -2., -2.));
  config.set("observation_max", VectorConstructor(1., 1.,  2.,  2.));
  config.set("action_min", VectorConstructor(-1, -1));
  config.set("action_max", VectorConstructor( 1, 1));
}

void MountainRegulatorTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  RegulatorTask::observe(state, obs, terminal);

  if (state.size() != 5)
  {
    ERROR("Received state size " << state.size() << ", expected 5");
    throw Exception("task/mountain/regulator requires dynamics/mountain");
  }
    
  obs->v.resize(4);
  for (size_t ii=0; ii < 4; ++ii)
    (*obs)[ii] = state[ii];
  obs->absorbing = false;
}

bool MountainRegulatorTask::invert(const Observation &obs, Vector *state, double time) const
{
  *state = extend(obs, VectorConstructor(time));
  
  return true;
}
