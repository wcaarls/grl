#ifndef GRL_ANN_PROJECTOR_H_
#define GRL_ANN_PROJECTOR_H_

#include <grl/projector.h>
#include <grl/projections/sample.h>

namespace grl {

class ANNProjector : public SampleProjector
{
  public:
    TYPEINFO("projector/sample/ann")
    
    struct SampleRef
    {
      size_t index;
      float dist;
      
      bool operator<(const SampleRef &rhs) const
      {
        return dist < rhs.dist;
      }
    };

  public:
    StorePtr store_;
    class ANNkd_tree *index_;
    size_t max_samples_, indexed_samples_, dims_, bucket_size_, neighbors_;
    double error_bound_;
    mutable ReadWriteLock rwlock_;

  public:
    ANNProjector() : index_(NULL), max_samples_(0), dims_(1), bucket_size_(10), neighbors_(20), error_bound_(0.1) { }
    void reindex();
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From SampleProjector
    virtual ANNProjector *clone() const;
    virtual void push(Sample *sample);
    virtual ProjectionPtr project(const Vector &in) const;
};

}

#endif /* GRL_ANN_PROJECTOR_H_ */
