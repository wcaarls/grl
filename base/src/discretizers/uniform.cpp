/** \file uniform.cpp
 * \brief Uniform discretizer source file.
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

#include <grl/discretizers/uniform.h>

using namespace grl;

REGISTER_CONFIGURABLE(UniformDiscretizer)

void UniformDiscretizer::request(ConfigurationRequest *config)
{
}

void UniformDiscretizer::configure(Configuration &config)
{
  Vector min = config["min"];
  Vector max = config["max"];
  Vector steps = config["steps"];
  
  Vector range, delta;

  range = max-min;
  delta = range/(steps-1);

  values_.resize(steps.size());

  for (size_t dd=0; dd < steps.size(); ++dd)
  {
    values_[dd].resize(steps[dd]);

    for (size_t vv=0; vv < steps[dd]; ++vv)
      values_[dd][vv] = min[dd] + delta[dd] * vv;
  }
}

void UniformDiscretizer::reconfigure(const Configuration &config)
{
}

UniformDiscretizer* UniformDiscretizer::clone()
{
  UniformDiscretizer *ud = new UniformDiscretizer();
  ud->values_ = values_;
  
  return ud;
}

void UniformDiscretizer::options(std::vector<Vector> *out) const
{
  size_t sz = 1;
  size_t dims = values_.size();

  std::vector<size_t> idx;
  idx.resize(dims, 0);

  for (size_t dd=0; dd < dims; ++dd)
    sz *= values_[dd].size();

  out->resize(sz);
  
  for (size_t ii=0; ii < sz; ++ii)
  {
    (*out)[ii].resize(dims);
    for (size_t dd=0; dd < dims; ++dd)
      (*out)[ii][dd] = values_[dd][idx[dd]];
    
    for (size_t dd=0; dd < dims; ++dd)
      if (++idx[dd] == values_[dd].size())
        idx[dd] = 0;
      else
        break;
  }
}
