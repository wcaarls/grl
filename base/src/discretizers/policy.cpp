/** \file policy.cpp
 * \brief Policy discretizer source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-09-01
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

#include <grl/discretizers/policy.h>

using namespace grl;

REGISTER_CONFIGURABLE(PolicyDiscretizer)

void PolicyDiscretizer::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("policy", "mapping/policy", "Policy whose action to return", policy_));
}

void PolicyDiscretizer::configure(Configuration &config)
{
  policy_ = (Policy*) config["policy"].ptr();
}

void PolicyDiscretizer::reconfigure(const Configuration &config)
{
}

PolicyDiscretizer::iterator PolicyDiscretizer::begin(const Vector &point) const
{
  return iterator(this, point, IndexVector::Constant(1, 0));
}

size_t PolicyDiscretizer::size() const
{
  return 1;
}

void PolicyDiscretizer::inc(iterator *it) const
{
  it->idx = IndexVector();
}

Vector PolicyDiscretizer::get(const iterator &it) const
{
  if (!it.idx.size())
    return Vector();

  Action action;
  policy_->act(it.point, &action);
  return action.v;
}

Vector PolicyDiscretizer::at(const Vector &point, size_t idx) const
{
  if (idx)
    return Vector();

  Action action;
  policy_->act(point, &action);
  return action.v;
}
