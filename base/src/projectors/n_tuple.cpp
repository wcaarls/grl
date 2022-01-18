/** \file n_tuple.cpp
 * \brief n-Tuple projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2022-01-17
 *
 * \copyright \verbatim
 * Copyright (c) 2022, Wouter Caarls
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

#include <iomanip>
#include <grl/projectors/n_tuple.h>

using namespace grl;

REGISTER_CONFIGURABLE(NTupleProjector)

void NTupleProjector::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("tuple_size", "Tuple size", tuple_size_));
  config->push_back(CRP("memory", "int.memory", "Hash table size", memory_));
  config->push_back(CRP("safe", "Collision detection (0=off, 1=claim on write, 2=claim always)", safe_, CRP::Configuration, 0, 2));
  config->push_back(CRP("resolution", "Size of encodings (0=split discriminators on dimension)", resolution_));
  
  if (role == "observation")
  {
    config->push_back(CRP("input_min", "vector.observation_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.observation_max", "Upper input dimension limit", max_, CRP::System));
  }
  else if (role == "action")
  {
    config->push_back(CRP("input_min", "vector.action_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.action_max", "Upper input dimension limit", max_, CRP::System));
  }
  else if (role == "pair")
  {
    config->push_back(CRP("input_min", "vector.observation_min++vector.action_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "vector.observation_max++vector.action_max", "Upper input dimension limit", max_, CRP::System));
  }
  else
  {
    config->push_back(CRP("input_min", "Lower input dimension limit", min_, CRP::System));
    config->push_back(CRP("input_max", "Upper input dimension limit", max_, CRP::System));
  }
}

void NTupleProjector::configure(Configuration &config)
{
  tuple_size_ = config["tuple_size"];
  memory_ = config["memory"];
  safe_ = config["safe"];
  
  if (safe_)
  {
    indices_ = new int32_t[memory_];
    for (size_t ii=0; ii < memory_; ++ii)
      indices_[ii] = -1;
  }
  
  min_ = config["input_min"].v();
  max_ = config["input_max"].v();
  
  if (min_.size() != max_.size())
    throw bad_param("projector/n_tuple:{input_min,input_max}");
  
  resolution_ = config["resolution"].v();
  if (resolution_.size() == 1)
    resolution_ = ConstantVector(min_.size(), resolution_[0]);
    
  if (resolution_.size() != min_.size())
    throw bad_param("projector/n_tuple:resolution");
 
  // Find total input size
  input_size_ = 0;
  splits_ = 0;
  for (size_t ii=0; ii != resolution_.size(); ++ii)
  {
    input_size_ += resolution_[ii];
    if (!resolution_[ii])
      splits_++;
  }
    
  tuples_ = ceil(input_size_ / (double)tuple_size_);
  
  NOTICE("Total retina size is " << input_size_ << " split into " << tuples_ << " tuples, with " << splits_ << " splitting dimension(s)");

  // Create permutation vector
  map_.resize(input_size_);
  for (size_t ii=0; ii != input_size_; ++ii)
    map_[ii] = ii;
  for (size_t ii=0; ii != input_size_; ++ii)
  {
    int temp = map_[ii], rnd = RandGen::getInteger(input_size_);
    map_[ii] = map_[rnd];
    map_[rnd] = temp;
  }
}

void NTupleProjector::reconfigure(const Configuration &config)
{
  if (config.has("action"))
    if (config["action"].str() == "reset")
    {
      // Unclaim all hash table entries
      if (indices_)
        for (size_t ii=0; ii < memory_; ++ii)
          indices_[ii] = -1;
          
      // Randomize permutation vector
      for (size_t ii=0; ii != input_size_; ++ii)
      {
        int temp = map_[ii], rnd = RandGen::getInteger(input_size_);
        map_[ii] = map_[rnd];
        map_[rnd] = temp;
      }
    }
}

NTupleProjector &NTupleProjector::copy(const Configurable &obj)
{
  const NTupleProjector &tc = dynamic_cast<const NTupleProjector&>(obj);

  if (indices_)
    memcpy(indices_, tc.indices_, memory_*sizeof(int32_t));
  
  return *this;
}

ProjectionPtr NTupleProjector::_project(const Vector &in, bool claim) const
{
  int blocks = ceil(tuple_size_/32.);
  int rest = tuple_size_%32;
  if (!rest) rest = 32;
  int coordinates[blocks + splits_ + 1];

  if (in.size() != resolution_.size())
  {
    ERROR("Input vector " << in << " does not match resolution vector " << resolution_);
    throw bad_param("projector/n_tuple:resolution");
  }
  
  // Create retina
  Eigen::ArrayXi retina = Eigen::ArrayXi::Constant(input_size_, 0);
  for (int ii = 0, nn = 0, ss = 0; ii < in.size(); nn += resolution_[ii], ++ii)
  {
    if (resolution_[ii])
    { 
      // Quantize
      int v = (int) floor(resolution_[ii]*(in[ii] - min_[ii])/(max_[ii]-min_[ii]));
    
      // Avoid under- and overflow
      v = std::min(std::max(v, 0), (int)resolution_[ii]);
    
      // Fill retina (thermometer)
      for (int jj=0; jj != v; ++jj)
        retina[nn+jj] = 1;
    }
    else
    {
      // Make sure different values in this dimension get hashed
      // in different elements, splitting the discriminator
      union {
        float f;
        int i;
      } inu = { .f = (float)in[ii] };
      
      coordinates[ss++] = inu.i;
    }
  }
  
  IndexProjection *p = new IndexProjection();
  p->indices.resize(tuples_);
  
  // Loop over tuples
  for (int ii=0, bb=0; ii < tuples_; ++ii)
  {
    // Loop over blocks within tuple, as hashing uses 32-bit integers
    for (int jj = 0; jj < blocks; ++jj)
    {
      // Each block is 32 bits, unless either
      // (a) this is the last block of the tuple, and the tuple is not a multiple of 32
      // (b) this is the last tuple of the retina, and the retina size is not a multiple
      //     of the tuple size
      unsigned int v = 0;
      for (int kk=0; kk < 32 && bb < input_size_ && (jj != blocks-1 || kk < rest); ++kk)
        v = (v<<1) + retina[map_[bb++]];
        
      coordinates[splits_ + jj] = v;
    }

    // Add tuple number so different tuples are hashed in different elements
    coordinates[splits_ + blocks] = ii;
    p->indices[ii] = getFeatureLocation(coordinates, blocks+1, claim);
  }
  
  return ProjectionPtr(p);
}
