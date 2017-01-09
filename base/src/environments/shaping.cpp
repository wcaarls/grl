/** \file shaping.cpp
 * \brief Pre-environment that adds reward shaping.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-06-02
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#include <iomanip>

#include <grl/environment.h>

using namespace grl;

REGISTER_CONFIGURABLE(ShapingEnvironment)

void ShapingEnvironment::request(ConfigurationRequest *config)
{
  config->push_back(CRP("environment", "environment", "Environment to inject noise into", environment_));
  config->push_back(CRP("shaping_function", "mapping", "Potential function over states", shaping_function_));

  config->push_back(CRP("gamma", "Discount factor", gamma_, CRP::Configuration, 0., 1.));
}

void ShapingEnvironment::configure(Configuration &config)
{
  environment_ = (Environment*)config["environment"].ptr();
  shaping_function_ = (Mapping*)config["shaping_function"].ptr();
  
  gamma_ = config["gamma"];
}

void ShapingEnvironment::reconfigure(const Configuration &config)
{
}

ShapingEnvironment &ShapingEnvironment::copy(const Configurable &obj)
{
  const ShapingEnvironment& se = dynamic_cast<const ShapingEnvironment&>(obj);

  prev_obs_ = se.prev_obs_;
  total_reward_ = se.total_reward_;
  
  return *this;
}
    
void ShapingEnvironment::start(int test, Vector *obs)
{
  environment_->start(test, obs);
  
  prev_obs_ = *obs;
  total_reward_ = 0;
}

double ShapingEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  double tau = environment_->step(action, obs, reward, terminal);
  total_reward_ += *reward;
  
  Vector v;
  *reward += gamma_*shaping_function_->read(*obs, &v) - shaping_function_->read(prev_obs_, &v);
  
  prev_obs_ = *obs;
  return tau;
}

void ShapingEnvironment::report(std::ostream &os) const
{
  os << std::setw(15) << total_reward_;
}
