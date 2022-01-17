/** \file n_tuple.h
 * \brief n-Tuple projector header file.
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

#ifndef GRL_N_TUPLE_PROJECTOR_H_
#define GRL_N_TUPLE_PROJECTOR_H_

#include <grl/projector.h>

namespace grl
{

/// Hashed n-tuple.
class NTupleProjector : public Projector
{
  public:
    TYPEINFO("projector/n_tuple", "Hashed n-tuple projector")
    
  protected:
    int tuple_size_, memory_, safe_;
    Vector min_, max_, resolution_;
    mutable int32_t *indices_, pressure_;
    Eigen::ArrayXi map_;
    int tuples_, input_size_;
    
  public:
    NTupleProjector() : tuple_size_(16), memory_(8*1024*1024), safe_(0), indices_(NULL), pressure_(0), tuples_(0), input_size_(0) { }
    
    ~NTupleProjector()
    {
      safe_delete_array(&indices_);
    }
  
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual NTupleProjector &copy(const Configurable &obj);

    // From Projector
    virtual ProjectionLifetime lifetime() const { return safe_>0?plUpdate:plIndefinite; }
    virtual ProjectionPtr project(const Vector &in) const
    {
      return _project(in, true);
    }
    virtual void project(const Vector &base, const std::vector<Vector> &variants, std::vector<ProjectionPtr> *out) const
    {
      // NOTE: safe_ = 1 assumes these types of multi-projections are read-only
      out->clear();
      for (size_t ii=0; ii < variants.size(); ++ii)
        out->push_back(_project(extend(base, variants[ii]), safe_>1));
    }
  protected:
    ProjectionPtr _project(const Vector &in, bool claim) const;
    
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

    unsigned int getFeatureLocation(const int *ints, const unsigned int num_ints, bool claim) const
    {
      unsigned int h = createHashSum(ints, num_ints, 449), ii = h % memory_;
      
      if (indices_)
      {
        size_t collisions = 0;
        
        while (indices_[ii] != h && indices_[ii] != -1)
        {
          collisions++;
          if (++ii >= memory_)
            ii = 0;
        }
        
        if (collisions > pressure_)
        {
          pressure_ = collisions;
          if (pressure_ > 7)
            WARNING("Memory pressure is high (" << pressure_ << "). Increase hash table size");
        }
            
        if (claim)
        {
          // NOTE: Race condition. In rare cases, we could be overwriting a claim
          // someone else made after we checked. Subsequent projections for our
          // competitor will then point to a different memory location. The same
          // principle applies when the memory is not claimed now: it might be
          // claimed by someone else later, at which point the location in
          // subsequent projections might change.
          indices_[ii] = h;
        }
      }
      
      return ii;
    }
};

}

#endif /* GRL_N_TUPLE_PROJECTOR_H_ */
