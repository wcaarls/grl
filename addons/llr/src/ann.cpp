#include <ANN/ANN.h>
#include <grl/projectors/ann.h>

using namespace grl;

REGISTER_CONFIGURABLE(ANNProjector)

void ANNProjector::request(ConfigurationRequest *config)
{
}

void ANNProjector::configure(Configuration &config)
{
  store_ = StorePtr(new SampleStore());
  
  max_samples_ = config["samples"];
  neighbors_ = config["neighbors"];
  bucket_size_ = config["bucket_size"];
  error_bound_ = config["error_bound"];
  dims_ = config["dims"];
  
  indexed_samples_ = 0;
}

void ANNProjector::reconfigure(const Configuration &config)
{
}

void ANNProjector::push(Sample *sample)
{
  rwlock_.writeLock();

  store_->push_back(sample);

  // Should be in a separate thread
  if ((store_->size() - indexed_samples_) > std::min(indexed_samples_, (size_t)100))
  {
    rwlock_.unlock();
    reindex();
  }
  else
    rwlock_.unlock();
}

void ANNProjector::reindex()
{
  // Create new pruned store
  StorePtr newstore = StorePtr(store_->prune(max_samples_));

  // Build new index using new store
  ANNkd_tree *newindex = new ANNkd_tree((ANNcoord**)newstore->samples(), newstore->size(), dims_, bucket_size_);
  
  {
    WriteGuard guard(rwlock_);
  
    store_ = newstore;
    safe_delete(&index_);
    index_ = newindex;
    indexed_samples_ = store_->size();
  }
}

ANNProjector *ANNProjector::clone() const
{
  return NULL;
}

ProjectionPtr ANNProjector::project(const Vector &in) const
{
  ReadGuard guard(rwlock_);

  grl_assert(in.size() == dims_);
  
  ANNcoord query[dims_];
  
  for (size_t ii=0; ii < dims_; ++ii)
    query[ii] = in[ii];
    
  size_t index_samples = std::min(neighbors_, indexed_samples_),
         linear_samples = store_->size()-indexed_samples_,
         available_samples = std::min(neighbors_, index_samples+linear_samples);
         
  SampleProjection *projection = new SampleProjection;
  projection->store = store_;
  projection->query = in;

  if (available_samples)
  {
    std::vector<SampleRef> refs(index_samples+linear_samples);
      
    if (index_samples)
    {
      // Search store using index
      ANNkd_tree_copy index(*index_);
      ANNidx nn_idx[neighbors_];
      ANNdist dd[neighbors_];
      
      index.annkSearch(query, index_samples, nn_idx, dd, error_bound_);
      
      for (size_t ii=0; ii < index_samples; ++ii)
      {
        if (nn_idx[ii] == ANN_NULL_IDX)
        {
          // Search failed
          return ProjectionPtr(projection);
        }
      
        refs[ii].index = nn_idx[ii];
        refs[ii].dist = dd[ii];
      }
    }

    // Search overflowing samples linearly
    for (size_t ii=0; ii < linear_samples; ++ii)
    {
      double dist=0;
      for (size_t dd=0; dd < dims_; ++dd)
        dist += pow((*store_)[indexed_samples_+ii]->in[dd] - in[dd], 2);
      
      refs[index_samples+ii].index = indexed_samples_+ii;
      refs[index_samples+ii].dist = dist;
    }
    
    std::sort(refs.begin(), refs.end());
    
    // Return ProjectionPtr pointing to current store
    projection->indices.resize(available_samples);
    projection->weights.resize(available_samples);
    
    double hSqr = refs[available_samples-1].dist;
    
    for (size_t ii=0; ii < available_samples; ++ii)
    {
      projection->indices[ii] = refs[ii].index;
      projection->weights[ii] = sqrt(exp(-refs[ii].dist/hSqr));
    }
  }

  return ProjectionPtr(projection);
}
