/** \file tile_coding.h
 * \brief Tile coding projector header file.
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

#ifndef GRL_TILE_CODING_PROJECTOR_H_
#define GRL_TILE_CODING_PROJECTOR_H_

#include <grl/projector.h>

namespace grl
{

/// Hashed tile coding.
class TileCodingProjector : public Projector
{
  public:
    TYPEINFO("projector/tile_coding")
    
  protected:
    int tilings_, memory_;
    Vector resolution_, scaling_, wrapping_;
    
  public:
    TileCodingProjector() : tilings_(16), memory_(8*1024*1024) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Projector
    virtual TileCodingProjector *clone() const;
    virtual ProjectionPtr project(const Vector &in) const;
    
  protected:
    // createHashSum() should return an unsigned integer type!
    unsigned int createHashSum(const int *ints, const unsigned int num_ints, unsigned int seed) const
    {
      // MurmurHash2, by Austin Appleby

      // Quote of external documentation:
      // It has a few limitations:
      // 1. It will not work incrementally.
      // 2. It will not produce the same results on little-endian and big-endian machines.
      // 3. It is 32-bits. See the websites for 64-bit implementations.

      // 'm' and 'r' are mixing constants generated offline.
      // They're not really 'magic', they just happen to work well.
      const unsigned int m = 0x5bd1e995;
      const int r = 24;

      // Initialize the hash to a 'random' value
      unsigned int h = seed ^ num_ints;

      // Mix 4 bytes at a time into the hash
      for (unsigned int i=0; i<num_ints; i++)
      {
        unsigned int k = (unsigned int)ints[i];

        k *= m;
        k ^= k >> r;
        k *= m;

        h *= m;
        h ^= k;
      }
      // Do a few final mixes of the hash
      h ^= h >> 13;
      h *= m;
      h ^= h >> 15;

      return h;
    }

    unsigned int getFeatureLocation(const int *ints, const unsigned int num_ints) const
    {
      return createHashSum(ints, num_ints, 449) % memory_;
    }
};

}

#endif /* GRL_TILE_CODING_PROJECTOR_H_ */
