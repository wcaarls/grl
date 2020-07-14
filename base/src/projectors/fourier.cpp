/** \file fourier.cpp
 * \brief Fourier basis function projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-09
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

#include <string.h>
#include <grl/projectors/fourier.h>

using namespace grl;

REGISTER_CONFIGURABLE(FourierProjector)

void FourierProjector::request(const std::string &role, ConfigurationRequest *config)
{
  if (role == "observation")
  {
    config->push_back(CRP("input_min", "vector.observation_min", "Lower input dimension limit (for scaling)", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.observation_max", "Upper input dimension limit (for scaling)", max_, CRP::System));
  }
  else if (role == "action")
  {
    config->push_back(CRP("input_min", "vector.action_min", "Lower input dimension limit (for scaling)", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.action_max", "Upper input dimension limit (for scaling)", max_, CRP::System));
  }
  else if (role == "pair")
  {
    config->push_back(CRP("input_min", "vector.observation_min++vector.action_min", "Lower input dimension limit (for scaling)", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.observation_max++vector.action_max", "Upper input dimension limit (for scaling)", max_, CRP::System));
  }
  else
  {
    config->push_back(CRP("input_min", "Lower input dimension limit (for scaling)", min_, CRP::System));
    config->push_back(CRP("input_max", "Upper input dimension limit (for scaling)", max_, CRP::System));
  }
  
  config->push_back(CRP("order", "Order of approximation (bases per dimension)", (int)order_, CRP::Configuration, 0, 256));
  
  std::vector<std::string> options;
  options.push_back("even");
  options.push_back("odd");
  
  config->push_back(CRP("parity", "Whether to use odd or even bases", parity_, CRP::Configuration, options));
  config->push_back(CRP("memory", "int.memory", "Feature vector size", CRP::Provided));
}

void FourierProjector::configure(Configuration &config)
{
  min_ = config["input_min"].v();
  max_ = config["input_max"].v();
  
  if (min_.size() != max_.size())
    throw bad_param("projector/fourier:{input_min,input_max}");

  scaling_ = ConstantVector(min_.size(), 1.)/(max_-min_);
  dims_ = min_.size();

  order_ = config["order"];
  
  config.set("memory", pow(order_+1, dims_));
  
  parity_ = config["parity"].str();
}

void FourierProjector::reconfigure(const Configuration &config)
{
}

ProjectionPtr FourierProjector::project(const Vector &in) const
{
  VectorProjection *p = new VectorProjection();
  double (*f)(double);
  
  if (parity_ == "even")
    f = cos;
  else
    f = sin;
  
  if (in.size() != dims_)
    throw bad_param("projector/fourier:{input_min,input_max}");

  // Scale between 0 and 1
  Vector sv = (in-min_)*scaling_;

  p->vector.resize(pow(order_+1, dims_));
  size_t ss[256];
  bzero(ss, dims_*sizeof(size_t));

  // Go through all permutations of orders on each dimension
  for (size_t ii=0; ii < p->vector.size(); ++ii)
  { 
    // Calculate activation
    double x=0;
    for (size_t dd=0; dd < dims_; ++dd)
      x += ss[dd]*sv[dd];
    
    p->vector[ii] = f(M_PI*x);
   
    for (size_t dd=0; dd < dims_; ++dd)
    {
      ss[dd] = ss[dd] + 1;
      if (ss[dd] > dims_)
        ss[dd] = 0;
      else
        break;
    }
  }  
  
  // Add nonzero DC component even for odd parity
  p->vector[0] = 1.;

  return ProjectionPtr(p);
}
