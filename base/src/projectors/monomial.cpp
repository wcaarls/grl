/** \file monomial.cpp
 * \brief Monomial basis function projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-11-24
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

#include <string.h>
#include <grl/projectors/monomial.h>

using namespace grl;

REGISTER_CONFIGURABLE(MonomialProjector)
REGISTER_CONFIGURABLE(PowerProjector)

void MonomialProjector::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("operating_input", "Origin", operating_input_, CRP::Configuration));
  config->push_back(CRP("degree", "Maximum degree of monomials", (int)degree_, CRP::Configuration, 0, 10));
  config->push_back(CRP("memory", "int.memory", "Feature vector size", CRP::Provided));
}

void MonomialProjector::configure(Configuration &config)
{
  degree_ = config["degree"];
  operating_input_ = config["operating_input"].v();
  
  memory_ = fact(degree_+operating_input_.size())/(fact(degree_)*fact(operating_input_.size()));
  
  config.set("memory", memory_);
}

void MonomialProjector::reconfigure(const Configuration &config)
{
}

ProjectionPtr MonomialProjector::project(const Vector &in) const
{
  if (operating_input_.size() != in.size())
    throw bad_param("projector/monomial:operating_input");

  VectorProjection *p = new VectorProjection();
  Vector oin = in-operating_input_;

  p->vector.resize(memory_);
  
  size_t ii=0;
  
  // Degree 0
  p->vector[ii++] = 1;
  
  if (degree_ > 0)
  {
    // Degree 1
    for (size_t dd=0; dd < in.size(); ++dd)
      p->vector[ii++] = oin[dd];
  
    size_t last=1;
    IndexVector count = IndexVector::Ones(in.size());
    
    // Other degrees
    for (size_t oo=2; oo <= degree_; ++oo)
    {
      size_t total=ii-last;
      last = ii;
      
      for (size_t dd=0; dd < in.size(); ++dd)
      {
        for (size_t jj=last-total; jj < last; ++jj)
          p->vector[ii++] = p->vector[jj]*oin[dd];
          
        total -= count[dd];
        count[dd] += total;
      }
    }
  }
  
  return ProjectionPtr(p);
}

// PowerProjector

void PowerProjector::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("operating_input", "Origin", operating_input_, CRP::Configuration));
  config->push_back(CRP("degree", "Degree to which variables are raised", degree_, CRP::Configuration, -DBL_MAX, DBL_MAX));

  config->push_back(CRP("memory", "int.memory", "Feature vector size", CRP::Provided));
}

void PowerProjector::configure(Configuration &config)
{
  operating_input_ = config["operating_input"].v();
  degree_ = config["degree"];
  
  config.set("memory", operating_input_.size());
}

void PowerProjector::reconfigure(const Configuration &config)
{
}

ProjectionPtr PowerProjector::project(const Vector &in) const
{
  if (operating_input_.size() != in.size())
    throw bad_param("projector/quadratic:operating_input");

  VectorProjection *p = new VectorProjection();
  Vector oin = in-operating_input_;

  p->vector.resize(in.size());
  for (size_t dd=0; dd < in.size(); ++dd)
    p->vector[dd] = pow(oin[dd], degree_);
  
  return ProjectionPtr(p);
}
