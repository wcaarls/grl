/** \file split.cpp
 * \brief Split discretizer source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-02-03
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

#include <grl/discretizers/split.h>

using namespace grl;

REGISTER_CONFIGURABLE(SplitDiscretizer)

void SplitDiscretizer::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("identify", "Identify active discretizer before (-1) or after (1) value", identify_, CRP::Configuration, -1, 1));

  config->push_back(CRP("discretizer1", "discretizer." + role, "First discretizer", discretizer_[0]));
  config->push_back(CRP("discretizer2", "discretizer." + role, "Second discretizer", discretizer_[1]));
}

void SplitDiscretizer::configure(Configuration &config)
{
  identify_ = config["identify"];

  discretizer_[0] = (Discretizer*)config["discretizer1"].ptr();
  discretizer_[1] = (Discretizer*)config["discretizer2"].ptr();
}

void SplitDiscretizer::reconfigure(const Configuration &config)
{
}

SplitDiscretizer* SplitDiscretizer::clone()
{
  SplitDiscretizer *sd = new SplitDiscretizer(*this);
  sd->discretizer_[0] = discretizer_[0]->clone();
  sd->discretizer_[1] = discretizer_[1]->clone();
  return sd;
}

SplitDiscretizer::iterator SplitDiscretizer::begin(const Vector &point) const
{
  IndexVector idx = discretizer_[0]->begin(point).idx;
  idx.conservativeResize(idx.size()+1);
  idx[idx.size()-1] = 0;

  return iterator(this, point, idx);
}

size_t SplitDiscretizer::size(const Vector &point) const
{
  return discretizer_[0]->size(point) + discretizer_[1]->size(point);
}

// Note implicit assumption that downstream discretizers discard extra dimensions
void SplitDiscretizer::inc(iterator *it) const
{
  if (!it->idx.size())
    return;
  
  int dd = it->idx[it->idx.size()-1];
  discretizer_[dd]->inc(it);
  if (!it->idx.size() && ++dd < discretizer_.size())
  {
    // Switch to next discretizer
    it->idx = discretizer_[dd]->begin(it->point).idx;
    it->idx.conservativeResize(it->idx.size()+1);
    it->idx[it->idx.size()-1] = dd;
  }
}

// Note implicit assumption that downstream discretizers discard extra dimensions
Vector SplitDiscretizer::get(const iterator &it) const
{
  if (!it.idx.size())
    return Vector();

  int dd = it.idx[it.idx.size()-1];
  
  Vector v = discretizer_[dd]->get(it);
  
  if (identify_)
  {
    Vector v2(v.size()+1);
    
    if (identify_ == -1)
      v2 << dd, v;
    else
      v2 << v, dd;
    
    return v2;
  }
  else
    return v;
}

Vector SplitDiscretizer::at(const Vector &point, size_t idx) const
{
  // Find active iterator
  int ii=0, dd=0;
  for (; ii <= idx; ii += discretizer_[dd++]->size(point));
  
  // Backtrack to start of active iterator
  ii -= discretizer_[--dd]->size(point);

  Vector v = discretizer_[dd]->at(point, idx-ii);
  
  if (identify_)
  {
    Vector v2(v.size()+1);
    
    if (identify_ == -1)
      v2 << dd, v;
    else
      v2 << v, dd;
    
    return v2;
  }
  else
    return v;
}
