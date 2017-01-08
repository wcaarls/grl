/** \file random.cpp
 * \brief Configurable class of RandGen source file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@ctudelft.nl>
 * \date      2016-06-27
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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
#include <grl/samplers/random_generator.h>
#include <grl/grl.h>

using namespace grl;

REGISTER_CONFIGURABLE(RandomGeneratorUniform)
REGISTER_CONFIGURABLE(RandomGeneratorUniformInteger)
REGISTER_CONFIGURABLE(RandomGeneratorNormal)
REGISTER_CONFIGURABLE(RandomGeneratorOrnsteinUhlenbeck)

void RandomGeneratorUniform::request(ConfigurationRequest *config)
{
  config->push_back(CRP("lower", "Lower bound of an interval", lower_, CRP::Configuration));
  config->push_back(CRP("upper", "Upper bound of an interval", upper_, CRP::Configuration));
}

void RandomGeneratorUniform::configure(Configuration &config)
{
  lower_ = config["lower"];
  upper_ = config["upper"];
}

void RandomGeneratorUniform::reconfigure(const Configuration &config)
{
}

double RandomGeneratorUniform::get(double *param) const
{
  return rand_->getUniform(lower_, upper_);
}

//////////////////////////////////

void RandomGeneratorUniformInteger::request(ConfigurationRequest *config)
{
  config->push_back(CRP("ma", "Upper bound of an interval [0, ma)", ma_, CRP::Configuration));
}

void RandomGeneratorUniformInteger::configure(Configuration &config)
{
  ma_ = config["ma"];
}

void RandomGeneratorUniformInteger::reconfigure(const Configuration &config)
{
}

double RandomGeneratorUniformInteger::get(double *param) const
{
  return rand_->getInteger(ma_);
}

//////////////////////////////////////

void RandomGeneratorNormal::request(ConfigurationRequest *config)
{
  config->push_back(CRP("mu", "Mean", mu_, CRP::Configuration));
  config->push_back(CRP("sigma", "Standart deviation", sigma_, CRP::Configuration));
}

void RandomGeneratorNormal::configure(Configuration &config)
{
  mu_ = config["mu"];
  sigma_ = config["sigma"];
}

void RandomGeneratorNormal::reconfigure(const Configuration &config)
{
}

double RandomGeneratorNormal::get(double *param) const
{
  return rand_->getNormal(mu_, sigma_);
}

//////////////////////////////////////

void RandomGeneratorOrnsteinUhlenbeck::request(ConfigurationRequest *config)
{
  config->push_back(CRP("center", "Attraction point", center_, CRP::Configuration));
  config->push_back(CRP("theta", "Theta", theta_, CRP::Configuration));
  config->push_back(CRP("sigma", "Sigma", sigma_, CRP::Configuration));
}

void RandomGeneratorOrnsteinUhlenbeck::configure(Configuration &config)
{
  center_ = config["center"];
  theta_ = config["theta"];
  sigma_ = config["sigma"];
}

void RandomGeneratorOrnsteinUhlenbeck::reconfigure(const Configuration &config)
{
}

double RandomGeneratorOrnsteinUhlenbeck::get(double *param) const
{
  grl_assert(param != NULL);
  return rand_->getOrnsteinUhlenbeck(*param, center_, theta_, sigma_);
}
