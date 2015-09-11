/** \file peaked.cpp
 * \brief Peaked projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-09-01
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
#include <grl/projectors/peaked.h>

using namespace grl;

REGISTER_CONFIGURABLE(PeakedProjector)

void PeakedProjector::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("peaking", "Extra resolution factor around center (offset by 1/factor at edges)", peaking_, CRP::Configuration));
  
  NormalizingProjector::request(role, config);
}

void PeakedProjector::configure(Configuration &config)
{
  NormalizingProjector::configure(config);
  
  peaking_ = config["peaking"];
  
  if (peaking_.size() != min_.size())
    throw bad_param("projector/peaked:peaking");

  scaling_ = 1./(max_-min_);
  range2_ = (max_-min_)/2.;
}

void PeakedProjector::reconfigure(const Configuration &config)
{
}

PeakedProjector *PeakedProjector::clone() const
{
  PeakedProjector *projector = new PeakedProjector(*this);
  projector->projector_ = projector_->clone();
  return projector;
}

ProjectionPtr PeakedProjector::project(const Vector &in) const
{
  // Scale input to [-1, 1], apply squashing, and rescale to range
  return projector_->project((squash(2.*(in-min_)*scaling_-1., peaking_)+1.)*range2_+min_);
}

Matrix PeakedProjector::jacobian(const Vector &in) const
{
  if (in.size() != peaking_.size())
    throw bad_param("projector/peaked:peaking"); 

  Vector diag(in.size());
  
  for (size_t ii=0; ii < in.size(); ++ii)
  {
    double f = peaking_[ii];
    if (f)
    {
      double x = in[ii], mi = min_[ii], s = scaling_[ii], r2 = range2_[ii], sf = sign(f), af1 = 1/fabs(f), axs = fabs(s*(2*mi - 2*x) + 1);
  
      diag[ii] = r2*((2*s*(sf/2 + af1 + 0.5))/(af1 - sf/2 + axs*sf + 0.5) - (2*s*axs*sf*(sf/2 + af1 + 0.5))/pow(af1 - sf/2 + axs*sf + 0.5, 2));
    }
    else
      diag[ii] = 1.;
  }
  
  return projector_->jacobian((squash(2.*(in-min_)*scaling_-1., peaking_)+1.)*range2_+min_)*Matrix::Diagonal(diag);
}
