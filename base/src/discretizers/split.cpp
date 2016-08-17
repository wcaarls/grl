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
  config->push_back(CRP("discretizer1", "discretizer." + role, "First discretizer", discretizer_[0]));
  config->push_back(CRP("discretizer2", "discretizer." + role, "Second discretizer", discretizer_[1]));
}

void SplitDiscretizer::configure(Configuration &config)
{
  discretizer_[0] = (Discretizer*)config["discretizer1"].ptr();
  discretizer_[1] = (Discretizer*)config["discretizer2"].ptr();
  
  iterator it1 = discretizer_[0]->begin(),
           it2 = discretizer_[1]->begin();
           
  idxsize_ = std::max(it1.idx().size(), it2.idx().size())+1;
  ressize_ = std::max((*it1).size(), (*it2).size())+1;
}

void SplitDiscretizer::reconfigure(const Configuration &config)
{
}

SplitDiscretizer* SplitDiscretizer::clone()
{
  SplitDiscretizer *sd = new SplitDiscretizer(*this);
  sd->discretizer_[0] = discretizer_[0]->clone();
  sd->discretizer_[1] = discretizer_[1]->clone();
}

SplitDiscretizer::iterator SplitDiscretizer::begin() const
{
  return iterator(this, IndexVector::Constant(idxsize_, 0));
}

size_t SplitDiscretizer::size() const
{
  return discretizer_[0]->size() + discretizer_[1]->size();
}

// Note implicit assumption that downstream discretizers discard extra dimensions
void SplitDiscretizer::inc(IndexVector *idx) const
{
  if (!idx->size())
    return;
    
  int dd = (*idx)[idxsize_-1];
  discretizer_[dd]->inc(idx);
  if (!idx->size() && dd < discretizer_.size()-1)
  {
    // Switch to next discretizer
    *idx = IndexVector::Constant(idxsize_, 0);
    (*idx)[idxsize_-1] = dd+1;
  }
}

// Note implicit assumption that downstream discretizers discard extra dimensions
Vector SplitDiscretizer::get(const IndexVector &idx) const
{
  if (!idx.size())
    return Vector();

  int dd = idx[idxsize_-1];
  
  Vector v(ressize_);
  v << dd, discretizer_[dd]->get(idx);

  return v;
}
