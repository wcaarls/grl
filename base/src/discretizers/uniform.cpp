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

void UniformDiscretizer::request(const std::string &role, ConfigurationRequest *config)
{
  if (role == "action")
  {
    config->push_back(CRP("min", "vector.action_min", "Lower limit", min_, CRP::System));
    config->push_back(CRP("max", "vector.action_max", "Upper limit", max_, CRP::System));
  }
  else if (role == "observation")
  {
    config->push_back(CRP("min", "vector.observation_min", "Lower limit", min_, CRP::System));
    config->push_back(CRP("max", "vector.observation_max", "Upper limit", max_, CRP::System));
  }
  else
  {
    config->push_back(CRP("min", "Lower limit", min_, CRP::System));
    config->push_back(CRP("max", "Upper limit", max_, CRP::System));
  }
  
  config->push_back(CRP("steps", "Discretization steps per dimension", steps_, CRP::Configuration));
}

void UniformDiscretizer::configure(Configuration &config)
{
  min_ = config["min"].v();
  max_ = config["max"].v();
  steps_ = config["steps"].v();
  
  if (min_.size() != max_.size() || min_.size() != steps_.size())
    throw bad_param("discretizer/uniform:{min,max,steps}");
  
  for (int ii = 0; ii < steps_.size(); ii++)
    if (steps_[ii] == 0)
      throw bad_param("discretizer/uniform:steps");

  Vector range, delta;  

  range = max_-min_;
  delta = range/(steps_-1);

  for (size_t dd=0; dd < steps_.size(); ++dd)
    if (std::isnan(delta[dd]))
      delta[dd] = 0.;

  values_.resize(steps_.size());

  for (size_t dd=0; dd < steps_.size(); ++dd)
  {
    if (steps_[dd] < 1)
      throw bad_param("discretizer/uniform:steps");
  
    values_[dd].resize((unsigned int)steps_[dd]);

    for (size_t vv=0; vv < steps_[dd]; ++vv)
      values_[dd][vv] = min_[dd] + delta[dd] * vv;
  }
}

void UniformDiscretizer::reconfigure(const Configuration &config)
{
}

UniformDiscretizer* UniformDiscretizer::clone()
{
  return new UniformDiscretizer(*this);
}

UniformDiscretizer::iterator UniformDiscretizer::begin() const
{
  return iterator(this, Vector(), IndexVector::Constant(steps_.size(), 0));
}

size_t UniformDiscretizer::size() const
{
  return prod(steps_);
}

void UniformDiscretizer::inc(iterator *it) const
{
  if (!it->idx.size())
    return;

  size_t dd;
  for (dd=0; dd < steps_.size(); ++dd)
    if (++it->idx[dd] == values_[dd].size())
      it->idx[dd] = 0;
    else
      break;
      
  if (dd == steps_.size())
    it->idx = IndexVector();
}

Vector UniformDiscretizer::get(const iterator &it) const
{
  if (!it.idx.size())
    return Vector();

  Vector out(steps_.size());
  for (size_t dd=0; dd < out.size(); ++dd)
    out[dd] = values_[dd][it.idx[dd]];
    
  return out;
}

Vector UniformDiscretizer::at(size_t idx) const
{
  Vector out(steps_.size());

  for (size_t dd=0; dd < steps_.size(); ++dd)
  {
    size_t ss = steps_[dd];

    out[dd] = values_[dd][idx % ss];
    idx /= ss;
  }

  return out;
}

size_t UniformDiscretizer::discretize(Vector &vec) const
{
  double steps = 1;
  double offset = 0;

  for (int dd=0; dd < steps_.size(); ++dd)
  {
    double nearest = values_[dd][0];
    size_t nearest_vv = 0;
    for (size_t vv=1; vv < steps_[dd]; ++vv)
    {
      if (fabs(nearest-vec[dd]) > fabs(values_[dd][vv]-vec[dd]))
      {
        nearest = values_[dd][vv];
        nearest_vv = vv;
      }
    }
    vec[dd] = nearest;

    // calculating offset
    offset += nearest_vv*steps;
    steps *= steps_[dd];
  }

  // offset = sample_[0] + sample_[1]*steps_[0] + sample_[2]*steps_[0]*steps_[1];
  return static_cast<size_t>(round(offset));
}



