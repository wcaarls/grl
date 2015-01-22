/** \file sample.h
 * \brief Sample-based projection header file.
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

#ifndef GRL_SAMPLE_PROJECTION_H_
#define GRL_SAMPLE_PROJECTION_H_

#include <grl/mutex.h>
#include <grl/projection.h>

#define SS_MAX_COORDS 16

namespace grl
{

/// Supervised learning sample (input-output pair with relevance).
class Sample
{
  friend class SampleStore;

  public:
    double in[SS_MAX_COORDS], out[SS_MAX_COORDS];
    double relevance;
    
  private:
    class SampleStore *owner;

  public:
    bool operator<(const Sample &obj) const
    {
      return relevance < obj.relevance;
    }
    
    static bool less(Sample* const &lhs, Sample* const& rhs)
    {
      return *lhs < *rhs;
    }
};

template<class T>
struct PointerLess
{
  bool operator()(T* const& lhs, T* const& rhs)
  {
    return *lhs < *rhs;
  }
};
                             
/// Sample database.
/**
 * Upon destruction, the database only deletes the samples which belong
 * to it. When the database is copied (pruned), ownership transfers.
 * Since samples store which database they use in a shared_ptr, it will
 * remain valid throughout their lifetimes.
 */
class SampleStore : public Lockable
{
  protected:
    std::vector<Sample*> samples_;
    ReadWriteLock rwlock_;
    
  public:
    SampleStore(size_t sz=10000)
    {
      samples_.reserve(sz);
    }
          
    ~SampleStore()
    {
      for (size_t ii=0; ii < samples_.size(); ++ii)
        if (samples_[ii]->owner == this)
          delete samples_[ii];
    }
  
    SampleStore *clone()
    {
      ReadGuard guard(rwlock_);
    
      SampleStore *ss = new SampleStore(samples_.size());
      
      // Deep copy
      for (size_t ii=0; ii < samples_.size(); ++ii)
      {
        ss->samples_.push_back(new Sample(*samples_[ii]));
        ss->samples_.back()->owner = this;
      }
        
      return ss;
    }
    
    SampleStore *prune(size_t max_samples)
    {
      ReadGuard guard(rwlock_);
      SampleStore *ss = new SampleStore(2*max_samples);
      
      // Shallow copy
      ss->samples_ = samples_;
      
      if (ss->size() > max_samples)
      {
        // Sort by relevance
        std::sort(ss->samples_.begin(), ss->samples_.end(), PointerLess<Sample>());
      
        // Drop pruned samples
        ss->samples_.resize(max_samples);
      }
       
      // Transfer ownership of remaining samples
      for (size_t ii=0; ii < ss->samples_.size(); ++ii)
        ss->samples_[ii]->owner = ss;
      
      return ss;
    }
  
    void push_back(Sample *sample)
    {
      WriteGuard guard(rwlock_);
      
      sample->owner = this;
      samples_.push_back(sample);
    }
    
    // Must be locked
    Sample *operator[](int idx)
    {
      return samples_[idx];
    }
    
    // Must be locked
    size_t size()
    {
      return samples_.size();
    }
    
    // Must be locked
    Sample **samples()
    {
      return samples_.data();
    }
    
    virtual void lock()
    {
      rwlock_.readLock();
    }
    
    virtual void unlock()
    {
      rwlock_.unlock();
    }
};

/// Shared pointer to a SampleStore.
typedef boost::shared_ptr<SampleStore> StorePtr;

/// Search vector plus neighbor indices and weights (e.g. ANNProjector for LLRRepresentation)
class SampleProjection : public Projection
{
  public:
    StorePtr store;
    Vector query;
    std::vector<size_t> indices;
    Vector weights;
    
    virtual SampleProjection *clone() const
    {
      return new SampleProjection(*this);
    }

    virtual void ssub(const Projection &rhs)
    {
      const SampleProjection &np = dynamic_cast<const SampleProjection&>(rhs);
      for (size_t ii=0; ii < indices.size(); ++ii)
        for (size_t jj=0; jj < np.indices.size(); ++jj)
          if (indices[ii] == np.indices[ii])
            weights[ii] = 0.;
    }
};

}

#endif /* GRL_SAMPLE_PROJECTION_H_ */
