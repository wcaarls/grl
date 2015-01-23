/** \file multisine.cpp
 * \brief Multisine mapping source file.
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

#include <grl/representations/multisine.h>

using namespace grl;

REGISTER_CONFIGURABLE(MultisineMapping)

void MultisineMapping::request(ConfigurationRequest *config)
{
  config->push_back(CRP("inputs", "Number of input dimensions", (int)inputs_));
  config->push_back(CRP("outputs", "Number of output dimensions", (int)outputs_));
  config->push_back(CRP("sines", "Number of sines", (int)sines_));
}

void MultisineMapping::configure(Configuration &config)
{
  outputs_ = config["outputs"];
  sines_ = config["sines"];
  inputs_ = config["inputs"];

  params_ = RandGen::getVector(p(outputs_, 0, 0, 0));

  // Choose some sensible scales  
  for (size_t oo=0; oo < outputs_; ++oo)
    for (size_t ss=0; ss < sines_; ++ss)
      for (size_t ii=0; ii < inputs_; ++ii)
        params_[p(oo, ss, ii, 1)] = M_PI+3*M_PI*params_[p(oo, ss, ii, 1)];
}

void MultisineMapping::reconfigure(const Configuration &config)
{
}

MultisineMapping *MultisineMapping::clone() const
{
  return new MultisineMapping(*this);
}

double MultisineMapping::read(const ProjectionPtr &projection, Vector *result) const
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  grl_assert(vp);

  result->resize(outputs_);
  
  for (size_t oo=0; oo < outputs_; ++oo)
  {
    double r = 0;
    
    for (size_t ss=0; ss < sines_; ++ss)
    {
      // Amplitude
      double rs = params_[p(oo, ss+1, 0, -1)];
      
      // Multiple by activation over each input
      for (size_t ii=0; ii < inputs_; ++ii)
        rs *= sin(params_[p(oo, ss, ii, 0)] + params_[p(oo, ss, ii, 1)]*vp->vector[ii]);

      // Add all sines together
      r += rs;
    }
        
    (*result)[oo] = r;
  }
  
  return (*result)[0];
}
