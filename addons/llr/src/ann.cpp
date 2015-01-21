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
}

void ANNProjector::reconfigure(const Configuration &config)
{
}

void ANNProjector::push(Sample *sample)
{
  WriteGuard guard(rwlock_);

  store_->push_back(sample);
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
  
  // Search store using index
  ANNkd_tree_copy index(*index_);
  ANNidx nn_idx[neighbors_];
  ANNdist dd[neighbors_];
  ANNcoord query[dims_];
  
  grl_assert(in.size() == dims_);
  
  for (size_t ii=0; ii < dims_; ++ii)
    query[ii] = in[ii];
  
  index.annkSearch(query, neighbors_, nn_idx, dd, error_bound_);
  
  // Search overflowing samples linearly
  for (size_t ii=indexed_samples_; ii < store_->size(); ++ii)
  {
    
  }
  
  // Return ProjectionPtr pointing to current store
  SampleProjection *projection = new SampleProjection;
  projection->store = store_;
  projection->query = in;
  projection->samples.resize(neighbors_);
  projection->weights.resize(neighbors_);
  
  double hSqr = pow(dd[neighbors_-1], 2);
  
  for (size_t ii=0; ii < neighbors_; ++ii)
  {
    projection->samples[ii] = nn_idx[ii];
    projection->weights[ii] = sqrt(exp(pow(dd[ii], 2)/hSqr));
  }
  
  return ProjectionPtr(projection);
}
