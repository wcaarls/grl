/*
 * tc_projector.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef TC_PROJECTOR_H_
#define TC_PROJECTOR_H_

#include <grl/projector.h>

namespace grl
{

/// Hashed tile coding.
class TileCodingProjector : public Projector
{
  public:
    TYPEINFO("projector/tilecoding")
    
  protected:
    int tilings_, memory_;
    Vector scaling_, wrapping_;
    
  public:
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

#endif /* TC_PROJECTOR_H_ */
