#ifndef GRL_ANN_PROJECTOR_H_
#define GRL_ANN_PROJECTOR_H_

#include <grl/projector.h>
#include <grl/projections/neighbor.h>

namespace grl {

class ANNProjector : public NeighborProjector
{
  public:
    TYPEINFO("projector/neighbor/ann")

  public:
    StorePtr store_;
    class ANNkd_tree *index_;
    size_t max_samples_, indexed_samples_, dims_, bucket_size_, neighbors_;
    double error_bound_;
    mutable ReadWriteLock rwlock_;

  public:
    void reindex();
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From NeighborProjector
    virtual ANNProjector *clone() const;
    virtual void push(Sample *sample);
    virtual ProjectionPtr project(const Vector &in) const;
};

}

#endif /* GRL_ANN_PROJECTOR_H_ */
