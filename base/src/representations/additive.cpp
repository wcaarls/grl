/** \file additive.cpp
 * \brief Additive representation source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-11-15
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

#include <grl/representations/additive.h>

using namespace grl;

REGISTER_CONFIGURABLE(AdditiveRepresentation)

void AdditiveRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("representation1", "representation." + role, "First representation", representation1_));
  config->push_back(CRP("representation2", "representation." + role, "Second representation", representation2_));
}

void AdditiveRepresentation::configure(Configuration &config)
{
  representation1_ = (Representation*)config["representation1"].ptr();
  representation2_ = (Representation*)config["representation2"].ptr();
}

void AdditiveRepresentation::reconfigure(const Configuration &config)
{
}

AdditiveRepresentation *AdditiveRepresentation::clone() const
{
  AdditiveRepresentation *ar = new AdditiveRepresentation(*this);
  ar->representation1_ = representation1_->clone();
  ar->representation2_ = representation2_->clone();
}

double AdditiveRepresentation::read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
{
  Vector res1, stddev1, res2, stddev2;
  representation1_->read(projection, &res1, &stddev1);
  representation2_->read(projection, &res2, &stddev2);
  
  *result = res1+res2;
  
  if (stddev)
    *stddev = sqrt(stddev1*stddev1+stddev2*stddev2);
  
  return (*result)[0];
}

void AdditiveRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  representation1_->write(projection, target, alpha);
  representation2_->write(projection, target, alpha);
}

void AdditiveRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  representation1_->write(projection, delta);
  representation2_->write(projection, delta);
}
