/** \file acrobot.cpp
 * \brief Acrobot environment source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-18
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

#include <grl/environments/acrobot.h>

using namespace grl;

REGISTER_CONFIGURABLE(AcrobotDynamics)
REGISTER_CONFIGURABLE(AcrobotBalancingTask)
REGISTER_CONFIGURABLE(AcrobotRegulatorTask)

void AcrobotDynamics::request(ConfigurationRequest *config)
{
}

void AcrobotDynamics::configure(Configuration &config)
{
}

void AcrobotDynamics::reconfigure(const Configuration &config)
{
}

void AcrobotDynamics::eom(const Vector &state, const Vector &action, Vector *xd) const
{
  if (state.size() != 5 || action.size() != 1)
    throw Exception("dynamics/acrobot requires a task/acrobot subclass");
    
  double l1 = 1, m1 = 1, m2 = 1, lc1 = 0.5, lc2 = 0.5, I1 = 1, I2 = 1, g = 9.8;
  double theta1 = state[siAngle1], theta2 = state[siAngle2],
         thetad1 = state[siAngleRate1], thetad2 = state[siAngleRate2];
  double tau = action[aiTau];
  
  double phi2 = m2*lc2*g*cos(theta1+theta2-M_PI/2);
  double phi1 = -m2*l1*lc2*thetad2*thetad2*sin(theta2)-2*m2*l1*lc2*thetad2*thetad1*sin(theta2) +
                (m1*lc1+m2*l1)*g*cos(theta1-M_PI/2)+phi2;
  double d2 = m2*(lc2*lc2+l1*lc2*cos(theta2))+I2;
  double d1 = m1*lc1*lc1 + m2*(l1*l1+lc2*lc2+2*l1*lc2*cos(theta2))+I1+I2;
  double thetadd2 = (tau+d2*phi1/d1-m2*l1*lc2*thetad2*thetad2*sin(theta2)-phi2)/
                    (m2*lc2*lc2+I2-d2*d2/d1);
  double thetadd1 = -(d2*thetadd2+phi1)/d1;
  
  // Limit velocity
  if (thetad1 >  4*M_PI) thetadd1 = fmin(thetadd1, 0);
  if (thetad1 < -4*M_PI) thetadd1 = fmax(thetadd1, 0);
  if (thetad2 >  9*M_PI) thetadd2 = fmin(thetadd2, 0);
  if (thetad2 < -9*M_PI) thetadd2 = fmax(thetadd2, 0);
  
  xd->resize(5);
  (*xd)[siAngle1] = thetad1;
  (*xd)[siAngle2] = thetad2;
  (*xd)[siAngleRate1] = thetadd1;
  (*xd)[siAngleRate2] = thetadd2;
  (*xd)[siTime] = 1;
}

void AcrobotBalancingTask::request(ConfigurationRequest *config)
{
  Task::request(config);
}

void AcrobotBalancingTask::configure(Configuration &config)
{
  config.set("observation_dims", 4);
  config.set("observation_min", VectorConstructor(M_PI-12*M_PI/180, -12*M_PI/180, -0.6, -1.1));
  config.set("observation_max", VectorConstructor(M_PI+12*M_PI/180,  12*M_PI/180,  0.6,  1.1));
  config.set("action_dims", 1);
  config.set("action_min", VectorConstructor(-1));
  config.set("action_max", VectorConstructor(1));
  config.set("reward_min", 1);
  config.set("reward_max", 1);
}

void AcrobotBalancingTask::reconfigure(const Configuration &config)
{
}

void AcrobotBalancingTask::start(int test, Vector *state) const
{
  *state = ConstantVector(5, 0.);
  (*state)[AcrobotDynamics::siAngle1] = M_PI+RandGen::get()*0.01-0.005;
  (*state)[AcrobotDynamics::siAngle2] = RandGen::get()*0.01-0.005;
}

void AcrobotBalancingTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 5)
    throw Exception("task/acrobot/balancing requires dynamics/acrobot");
    
  obs->resize(4);
  for (size_t ii=0; ii < 4; ++ii)
    (*obs)[ii] = state[ii];

  if (failed(state))
    *terminal = 2;
  else
    *terminal = state[AcrobotDynamics::siTime] > 20;
}

void AcrobotBalancingTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  if (state.size() != 5 || action.size() != 1 || next.size() != 5)
    throw Exception("task/acrobot/balancing requires dynamics/acrobot");

  *reward = !failed(next);
}

bool AcrobotBalancingTask::invert(const Vector &obs, Vector *state) const
{
  *state = VectorConstructor(obs[AcrobotDynamics::siAngle1],
                             obs[AcrobotDynamics::siAngle2],
                             obs[AcrobotDynamics::siAngleRate1],
                             obs[AcrobotDynamics::siAngleRate2],
                             0.);

  return true;
}

bool AcrobotBalancingTask::failed(const Vector &state) const
{
  return fabs(state[AcrobotDynamics::siAngle1]-M_PI) > 12*M_PI/180 ||
         fabs(state[AcrobotDynamics::siAngle2]) > 12*M_PI/180;
}

// Regulator

void AcrobotRegulatorTask::request(ConfigurationRequest *config)
{
  RegulatorTask::request(config);
}

void AcrobotRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  if (q_.size() != 4)
    throw bad_param("task/acrobot/regulator:q");
  if (r_.size() != 1)
    throw bad_param("task/acrobot/regulator:r");

  config.set("observation_min", VectorConstructor(0,      -M_PI, -4*M_PI, -9*M_PI));
  config.set("observation_max", VectorConstructor(2*M_PI,  M_PI,  4*M_PI,  9*M_PI));
  config.set("action_min", VectorConstructor(-1));
  config.set("action_max", VectorConstructor(1));
}

void AcrobotRegulatorTask::reconfigure(const Configuration &config)
{
  RegulatorTask::reconfigure(config);
}

void AcrobotRegulatorTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  if (state.size() != 5)
    throw Exception("task/acrobot/regulator requires dynamics/acrobot");
    
  obs->resize(4);
  for (size_t ii=0; ii < 4; ++ii)
    (*obs)[ii] = state[ii];

  *terminal = state[AcrobotDynamics::siTime] > 20;
}

bool AcrobotRegulatorTask::invert(const Vector &obs, Vector *state) const
{
  *state = VectorConstructor(obs[AcrobotDynamics::siAngle1],
                             obs[AcrobotDynamics::siAngle2],
                             obs[AcrobotDynamics::siAngleRate1],
                             obs[AcrobotDynamics::siAngleRate2],
                             0.);

  return true;
}
