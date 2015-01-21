/*
 * sample.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef GRL_SAMPLE_PROJECTION_H_
#define GRL_SAMPLE_PROJECTION_H_

#include <grl/mutex.h>
#include <grl/projection.h>

#define SS_MAX_COORDS 16

namespace grl
{

class Sample
{
  friend class SampleStore;

  public:
    float in[SS_MAX_COORDS], out[SS_MAX_COORDS];
    float relevance;
    
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
                             
class SampleStore : public Lockable
{
  protected:
    std::vector<Sample*> samples_;
    ReadWriteLock rwlock_;
    
  public:
    SampleStore()
    {
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
    
      SampleStore *ss = new SampleStore();
      
      // Deep copy
      ss->samples_.reserve(samples_.size());
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
      SampleStore *ss = new SampleStore();
      
      std::cout << "prune to " << max_samples << std::endl;
      
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

typedef boost::shared_ptr<SampleStore> StorePtr;

/// Search vector plus neighbor indices and distances (e.g. ANNProjector for LLRRepresentation)
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
