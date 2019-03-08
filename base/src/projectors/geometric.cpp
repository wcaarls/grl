/** \file geometric.cpp
 * \brief Geometric projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2019-02-25
 *
 * \copyright \verbatim
 * Copyright (c) 2019, Wouter Caarls
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
#include <grl/projectors/geometric.h>

using namespace grl;

REGISTER_CONFIGURABLE(GeometricProjector)

void GeometricProjector::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("angles", "vector", "[0, 1] vector that specifies which dimensions to convert", angles_));
  config->push_back(CRP("normalized", "int", "Assume angles are normalized to [0, 1] (1) or [-1, 1] (-1)", normalized_, CRP::Configuration, -1, 1));

  config->push_back(CRP("projector", "projector." + role, "Downstream vector projector", projector_));
  
  config->push_back(CRP("memory", "int.memory", "Feature vector size after conversion", CRP::Provided));
}

void GeometricProjector::configure(Configuration &config)
{
  projector_ = (Projector*) config["projector"].ptr();
  angles_ = config["angles"].v();
  normalized_ = config["normalized"];

  for (size_t ii=0; ii != angles_.size(); ++ii)
    if (angles_[ii] != 0 && angles_[ii] != 1)
      throw bad_param("projector/pre/geometric:angles");
      
  memory_ = angles_.size() + sum(angles_);
  
  switch (normalized_)
  {
    case -1:
      scaling_ =   M_PI; break;
    case  1:
      scaling_ = 2*M_PI; break;
    default:
      scaling_ = 1;
  }      
  
  config.set("memory", memory_);
}

void GeometricProjector::reconfigure(const Configuration &config)
{
}

ProjectionPtr GeometricProjector::project(const Vector &in) const
{
  if (in.size() != angles_.size())
    throw bad_param("projector/pre/geometric:angles"); 
  
  Vector v(memory_);

  for (size_t ii=0, jj=0; ii != angles_.size(); ++ii)
    if (angles_[ii])
    {
      v[jj++] = sin(in[ii]*scaling_);
      v[jj++] = cos(in[ii]*scaling_);
    }
    else
      v[jj++] = in[ii];
      
  return projector_->project(v);
}
