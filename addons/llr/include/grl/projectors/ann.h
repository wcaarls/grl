/** \file ann.h
 * \brief Approximate nearest neighbor projector header file.
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

#ifndef GRL_ANN_PROJECTOR_H_
#define GRL_ANN_PROJECTOR_H_

#include <grl/projector.h>
#include <grl/projections/sample.h>

namespace grl {

/// Projects onto samples found through approximate nearest-neighbor search.
class ANNProjector : public SampleProjector
{
  public:
    TYPEINFO("projector/sample/ann", "Projects onto samples found through approximate nearest-neighbor search")
    
    struct SampleRef
    {
      Sample *sample;
      float dist;
      
      bool operator<(const SampleRef &rhs) const
      {
        return dist < rhs.dist;
      }
    };

  public:
    StorePtr store_;
    class ANNkd_tree *index_;
    size_t max_samples_, indexed_samples_, dims_, bucket_size_, neighbors_, interval_, incremental_;
    double error_bound_, locality_;
    mutable ReadWriteLock rwlock_;

  public:
    ANNProjector() : index_(NULL), max_samples_(1000), dims_(1), bucket_size_(10), neighbors_(20), interval_(10), incremental_(1), error_bound_(0.01), locality_(1.) { }
    void reindex();
    
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual ANNProjector &copy(const Configurable &obj);
    
    // From SampleProjector
    virtual ProjectionLifetime lifetime() const { return plWrite; }
    virtual void push(Sample *sample);
    virtual StorePtr store() { ReadGuard guard(rwlock_); return store_; }
    virtual ProjectionPtr project(const Vector &in) const;
    virtual void finalize();
};

}

#endif /* GRL_ANN_PROJECTOR_H_ */
