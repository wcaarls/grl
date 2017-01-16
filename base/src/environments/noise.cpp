/** \file noise.cpp
 * \brief Pre-environment that injects noise.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-12-18
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

REGISTER_CONFIGURABLE(NoiseEnvironment)

void NoiseEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("environment", "environment", "Environment to inject noise into", environment_));

  config->push_back(CRP("sensor_noise", "Additive sensor noise standard deviation", sensor_noise_));
  config->push_back(CRP("actuator_noise", "Additive actuator noise standard deviation", actuator_noise_));
}

void NoiseEnvironment::configure(Configuration &config)
{
  environment_ = (Environment*)config["environment"].ptr();
  sensor_noise_ = config["sensor_noise"].v();
  actuator_noise_ = config["actuator_noise"].v();
}

void NoiseEnvironment::reconfigure(const Configuration &config)
{
}
    
void NoiseEnvironment::start(int test, Observation *obs)
{
  Rand *rand = RandGen::instance();

  environment_->start(test, obs);
  
  if (sensor_noise_.size() == 1 && obs->size() > 1)
    sensor_noise_ = ConstantVector(obs->size(), sensor_noise_[0]);
    
  if (sensor_noise_.size() != obs->size())
    throw bad_param("environment/pre/noise:sensor_noise");
    
  for (size_t ii=0; ii < sensor_noise_.size(); ++ii)
    (*obs)[ii] += rand->getNormal(0, sensor_noise_[ii]);
}

double NoiseEnvironment::step(const Action &action, Observation *obs, double *reward, int *terminal)
{
  Rand *rand = RandGen::instance();

  if (actuator_noise_.size() == 1 && action.size() > 1)
    actuator_noise_ = ConstantVector(action.size(), actuator_noise_[0]);
    
  if (actuator_noise_.size() != action.size())
    throw bad_param("environment/pre/noise:actuator_noise");
    
  Vector a(actuator_noise_.size());
  for (size_t ii=0; ii < actuator_noise_.size(); ++ii)
    a[ii] = action[ii] + rand->getNormal(0, actuator_noise_[ii]);

  double tau = environment_->step(a, obs, reward, terminal);
  
  if (sensor_noise_.size() != obs->size())
    throw bad_param("environment/pre/noise:sensor_noise");
    
  for (size_t ii=0; ii < sensor_noise_.size(); ++ii)
    (*obs)[ii] += rand->getNormal(0, sensor_noise_[ii]);

  return tau;
}
