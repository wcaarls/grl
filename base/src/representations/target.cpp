/** \file target.cpp
 * \brief Target representation source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-07-13
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
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

#include <grl/stacktrace.h>
#include <grl/representations/target.h>

using namespace grl;

REGISTER_CONFIGURABLE(TargetRepresentation)

void TargetRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("representation", "representation/parameterized." + role, "Downstream representation", representation_));
  config->push_back(CRP("interval", "Update interval (number of writes; 0=never update, <0=exp.mov.av.)", interval_, CRP::Configuration, 0., DBL_MAX));
  
  config->push_back(CRP("target", "representation/parameterized." + role, "Target representation", CRP::Provided));
}

void TargetRepresentation::configure(Configuration &config)
{
  representation_ = (ParameterizedRepresentation*)config["representation"].ptr();
  interval_ = config["interval"];
  
  target_representation_ = (ParameterizedRepresentation*)representation_->clone();
  config.set("target", target_representation_);
  
  synchronize();
}

void TargetRepresentation::reconfigure(const Configuration &config)
{
  synchronize();
}

void TargetRepresentation::synchronize()
{
  if (interval_ < 1)
  {
    LargeVector tparams = target_representation_->params(),
                params  = representation_->params();
    
    target_representation_->setParams(interval_*params + (1-interval_)*tparams);
  }
  else
  {
    TRACE("Synchronizing target representation");
    target_representation_->setParams(representation_->params());
    count_ = 0;
  }
}

void TargetRepresentation::checkSynchronize()
{
  count_++;
  if (interval_ && (interval_ < 1 || count_ >= interval_))
    synchronize();
}
