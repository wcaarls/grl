/** \file policy.cpp
 * \brief Policy visualization source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-14
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

#include <grl/visualizations/policy.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(PolicyVisualization) 

void PolicyVisualization::request(ConfigurationRequest *config)
{
  FieldVisualization::request(config);

  config->push_back(CRP("policy", "policy", "Control policy", policy_));
  
  config->push_back(CRP("dim", "Action dimension to visualize", (int)dim_, CRP::Online, 0));
}

void PolicyVisualization::configure(Configuration &config)
{
  FieldVisualization::configure(config);
  
  policy_ = (Policy*)config["policy"].ptr();
  
  dim_ = config["dim"];
  
  // Create window  
  create("Policy");
  
  // Let's get this show on the road
  start();
}

void PolicyVisualization::reconfigure(const Configuration &config)
{
}

double PolicyVisualization::value(const Vector &in) const
{
  Vector action, q;
  policy_->act(in, &action);
  
  return action[dim_];
}
