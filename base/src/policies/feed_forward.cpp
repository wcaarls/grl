/** \file feed_forward.cpp
 * \brief Feed forward policy source file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@gmail.com>
 * \date      2016-02-11
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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

#include <grl/policies/feed_forward.h>

using namespace grl;

REGISTER_CONFIGURABLE(FeedForwardPolicy)

void FeedForwardPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("controls", "mapping", "Maps time to controls", controls_, CRP::Configuration));
}

void FeedForwardPolicy::configure(Configuration &config)
{
  controls_ = (Mapping*)config["controls"].ptr();
}

void FeedForwardPolicy::reconfigure(const Configuration &config)
{
}

void FeedForwardPolicy::act(double time, const Observation &in, Action *out)
{
  controls_->read(VectorConstructor(time), &out->v);
  out->type = atGreedy;
}
