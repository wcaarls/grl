
/** \file value_function.cpp
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

#include <grl/visualizations/value_function.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(ValueFunctionVisualization) 

void ValueFunctionVisualization::request(ConfigurationRequest *config)
{
  FieldVisualization::request(config);

  config->push_back(CRP("projector", "projector", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation", "Q-value representation", representation_));
  config->push_back(CRP("policy", "policy/discrete/q", "Q-value based control policy", policy_));
}

void ValueFunctionVisualization::configure(Configuration &config)
{
  FieldVisualization::configure(config);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  policy_ = (QPolicy*)config["policy"].ptr();
  
  // Create window  
  create("Value function");
  
  // Let's get this show on the road
  start();
}

void ValueFunctionVisualization::reconfigure(const Configuration &config)
{
}

double ValueFunctionVisualization::value(const Vector &in) const
{
  Vector action, q;
  policy_->act(in, &action);
  return representation_->read(projector_->project(in, action), &q);
}
