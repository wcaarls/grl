/** \file value.cpp
 * \brief Value function visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#include <GL/gl.h>
#include <GL/glu.h>

#include <grl/visualizations/value.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(ValueVisualization) 
REGISTER_CONFIGURABLE(PolicyValueVisualization)

void ValueVisualization::request(ConfigurationRequest *config)
{
  FieldVisualization::request(config);
  
  config->push_back(CRP("output_dim", "Output dimension to visualize", (int)dim_, CRP::Online, 0));

  config->push_back(CRP("projector", "projector", "Projects inputs onto representation space", projector_));
  config->push_back(CRP("representation", "representation", "Value representation", representation_));
}

void ValueVisualization::configure(Configuration &config)
{
  FieldVisualization::configure(config);
  
  dim_ = config["output_dim"];
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  
  // Create window  
  create(path().c_str());
  
  // Let's get this show on the road
  start();
}

void ValueVisualization::reconfigure(const Configuration &config)
{
  FieldVisualization::reconfigure(config);
  
  config.get("output_dim", dim_);
}

double ValueVisualization::value(const Vector &in) const
{
  Vector v;
  representation_->read(projector_->project(in), &v);
  if (!v.size())
    return 0;
  else
    return v[dim_];
}

void PolicyValueVisualization::request(ConfigurationRequest *config)
{
  FieldVisualization::request(config);

  config->push_back(CRP("policy", "mapping/policy/discrete/q", "Q-value based control policy", policy_));
}

void PolicyValueVisualization::configure(Configuration &config)
{
  FieldVisualization::configure(config);
  
  policy_ = (QPolicy*)config["policy"].ptr();
  
  // Create window  
  create(path().c_str());
  
  // Let's get this show on the road
  start();
}

void PolicyValueVisualization::reconfigure(const Configuration &config)
{
  FieldVisualization::reconfigure(config);
}

double PolicyValueVisualization::value(const Vector &in) const
{
  return policy_->value(in);
}
