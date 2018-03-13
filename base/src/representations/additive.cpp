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
  config->push_back(CRP("learning", "Which representation to learn (-1=all)", learning_, CRP::Configuration, -1));

  config->push_back(CRP("representation", "representation." + role, "Downstream representations", &representation_));
}

void AdditiveRepresentation::configure(Configuration &config)
{
  learning_ = config["learning"];
  representation_ = *(ConfigurableList*)config["representation"].ptr();
  
  if (representation_.size() < 1)
    throw bad_param("representation/additive:representation");
}

void AdditiveRepresentation::reconfigure(const Configuration &config)
{
}

double AdditiveRepresentation::read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
{
  Vector newres, newstddev;

  representation_[0]->read(projection, result, stddev);
  if (stddev)
    *stddev *= *stddev;
  
  for (size_t ii=1; ii != representation_.size(); ++ii)
  {
    representation_[ii]->read(projection, &newres, &newstddev);
    
    *result += newres;
    if (stddev)
      *stddev += newstddev * newstddev;
  }
  
  if (stddev)
    *stddev = sqrt(*stddev);
  
  return (*result)[0];
}

void AdditiveRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  if (learning_ < 0 || representation_.size() < 2)
  {
    // Write to all downstream representations
    for (size_t ii=0; ii != representation_.size(); ++ii)
      representation_[ii]->write(projection, target/representation_.size(), alpha/representation_.size());
  }
  else if (learning_ < representation_.size())
  {
    // Write to only a specific representation
    Vector newtarget = target, v;
    for (size_t ii=0; ii != representation_.size(); ++ii)
      if (ii != learning_)
      {
        representation_[ii]->read(projection, &v);
        newtarget -= v;
      }
    
    representation_[learning_]->write(projection, newtarget, alpha);
  }
  else
  {
    ERROR("Learning representation " << learning_ << " out of bounds");
    throw bad_param("representation/additive:learning");
  }
}

void AdditiveRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  if (learning_ < 0)
  {
    // Update all downstream representations
    for (size_t ii=0; ii != representation_.size(); ++ii)
      representation_[ii]->update(projection, delta/representation_.size());
  }
  else if (learning_ < representation_.size())
  {
    // Update only a specific representation
    representation_[learning_]->update(projection, delta);
  }
  else
  {
    ERROR("Learning representation " << learning_ << " out of bounds");
    throw bad_param("representation/additive:learning");
  }
}
