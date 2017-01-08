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

  config->push_back(CRP("learning", "Which representation to learn (0=both)", learning_, CRP::Configuration, 0, 2));
}

void AdditiveRepresentation::configure(Configuration &config)
{
  representation1_ = (Representation*)config["representation1"].ptr();
  representation2_ = (Representation*)config["representation2"].ptr();
  learning_ = config["learning"];
}

void AdditiveRepresentation::reconfigure(const Configuration &config)
{
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
  Vector v;

  switch (learning_)
  {
    case 0:
      representation1_->write(projection, target/2, alpha/2);
      representation2_->write(projection, target/2, alpha/2);
      break;
    case 1:
      representation2_->read(projection, &v);
      representation1_->write(projection, target-v, alpha);
      break;
    case 2:
      representation1_->read(projection, &v);
      representation2_->write(projection, target-v, alpha);
      break;
  }
}

void AdditiveRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  switch (learning_)
  {
    case 0:
      representation1_->update(projection, delta/2);
      representation2_->update(projection, delta/2);
      break;
    case 1:
      representation1_->update(projection, delta);
      break;
    case 2:
      representation2_->update(projection, delta);
      break;
  }
}
