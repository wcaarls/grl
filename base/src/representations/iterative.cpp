/** \file dmp.cpp
 * \brief Iterative representation source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-07-03
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

#include <grl/representations/iterative.h>

using namespace grl;

REGISTER_CONFIGURABLE(IterativeRepresentation)

void IterativeRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("epochs", "Learning epochs", epochs_, CRP::Configuration));
  config->push_back(CRP("cumulative", "Add to training set instead of replacing it", cumulative_, CRP::Configuration, 0, 1));

  config->push_back(CRP("representation", "representation." + role, "Downstream representation", representation_));
}

void IterativeRepresentation::configure(Configuration &config)
{
  epochs_ = config["epochs"];
  cumulative_ = config["cumulative"];
  representation_ = (Representation*)config["representation"].ptr();
  
  reset();
}

void IterativeRepresentation::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    size_t lastsize = samples_.size();
    samples_.clear();
    samples_.reserve(lastsize);
  }
}

// TODO: should really make a different write() for alpha=1
void IterativeRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  if (prod(alpha) != 1)
    throw Exception("Iterative representation can not be written partially");
    
  samples_.push_back(Sample(projection, target));
}

void IterativeRepresentation::finalize()
{
  if (samples_.empty())
    return;

  Vector alpha = ConstantVector(samples_[0].second.size(), 1.);

  // TODO: break depending on progress
  for (size_t ii=0; ii < epochs_; ++ii)
  {
    CRAWL("Epoch " << ii);
  
    for (size_t jj=0; jj < samples_.size(); ++jj)
      representation_->write(samples_[jj].first, samples_[jj].second, alpha);
    representation_->finalize();
  }
  
  if (!cumulative_)
  {
    size_t lastsize = samples_.size();
    samples_.clear();
    samples_.reserve(lastsize);
  }
}
