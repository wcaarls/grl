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

void MonomialProjector::request(const std::string &role, ConfigurationRequest *config)
{
  if (role == "observation")
    config->push_back(CRP("inputs", "int.observation_dims", "Number of input dimensions", (int)inputs_, CRP::System));
  else if (role == "action")
    config->push_back(CRP("inputs", "int.action_dims", "Number of input dimensions", (int)inputs_, CRP::System));
  else if (role == "pair")
    config->push_back(CRP("inputs", "int.observation_dims+int.action_dims", "Number of input dimensions", (int)inputs_, CRP::System));
  else
    config->push_back(CRP("inputs", "Number of input dimensions", (int)inputs_, CRP::System));

  config->push_back(CRP("degree", "Maximum degree of monomials", (int)degree_, CRP::Configuration, 0, 4));
  
  config->push_back(CRP("memory", "int.memory", "Feature vector size", CRP::Provided));
}

void MonomialProjector::configure(Configuration &config)
{
  inputs_ = config["inputs"];
  degree_ = config["degree"];
  memory_ = fact(degree_+inputs_)/(fact(degree_)*fact(inputs_));
  
  config.set("memory", memory_);
}

void MonomialProjector::reconfigure(const Configuration &config)
{
}

MonomialProjector *MonomialProjector::clone() const
{
  return new MonomialProjector(*this);
}

ProjectionPtr MonomialProjector::project(const Vector &in) const
{
  VectorProjection *p = new VectorProjection();
  
  if (in.size() != inputs_)
    throw bad_param("projector/monomial:inputs");

  p->vector.resize(memory_);
  
  IndexVector expv = IndexVector::Zero(inputs_);

  size_t sum=0;
  for (size_t ii=0; ii < memory_; ++ii)
  {
    double total=1;
    for (size_t dd=0; dd < inputs_; ++dd)
      total = total * pow(in[dd], expv[dd]);
    p->vector[ii] = total;
    
    for (size_t dd=0; dd < inputs_; ++dd)
    {
      sum++;
      expv[dd]++;
      
      if (sum <= degree_)
        break;
      
      sum = sum - expv[dd];
      expv[dd] = 0;
    }
  };

  return ProjectionPtr(p);
}
