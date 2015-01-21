#include <grl/representations/linear.h>

using namespace grl;

REGISTER_CONFIGURABLE(LinearRepresentation)

void LinearRepresentation::request(ConfigurationRequest *config)
{
}

void LinearRepresentation::configure(Configuration &config)
{
  size_t memory;
  config.get("memory", memory, (size_t)16);
  config.get("outputs", outputs_, (size_t)1);
  
  Vector min, max;
  min.resize(outputs_, 0.);
  max.resize(outputs_, 1.);
  
  config.get("min", min);
  config.get("max", max);

  grl_assert(min.size() == outputs_);
  grl_assert(max.size() == outputs_);

  params_.resize(memory * outputs_ * 1024*1024);
  
  // Initialize memory
  Rand *rand = RandGen::instance();
  for (size_t ii=0; ii < memory*1024*1024; ++ii)
    for (size_t jj=0; jj < outputs_; ++jj)
      params_[ii*outputs_+jj] = rand->getUniform(min[jj], max[jj]);
}

void LinearRepresentation::reconfigure(const Configuration &config)
{
}

LinearRepresentation *LinearRepresentation::clone() const
{
  LinearRepresentation *lr = new LinearRepresentation();
  lr->params_ = params_;
  return lr;
}

double LinearRepresentation::read(const ProjectionPtr &projection, Vector *result) const
{
  Projection &p = *projection;
  
  IndexProjection *ip = dynamic_cast<IndexProjection*>(&p);
  if (ip)
  {
    result->clear();
    result->resize(outputs_, 0);
    
    for (size_t ii=0; ii < ip->indices.size(); ++ii)
      for (size_t jj=0; jj < outputs_; ++jj)
        (*result)[jj] += params_[ip->indices[ii]*outputs_+jj];
    for (size_t jj=0; jj < outputs_; ++jj)
      (*result)[jj] /= ip->indices.size();
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(&p);
    if (vp)
    {
      result->clear();
      result->resize(outputs_, 0);
      
      double activation=0;
      for (size_t ii=0; ii < vp->vector.size(); ++ii)
      {
        activation += vp->vector[ii];
        for (size_t jj=0; jj < outputs_; ++jj)
          (*result)[jj] += params_[ii*outputs_+jj]*vp->vector[ii];
      }
      for (size_t jj=0; jj < outputs_; ++jj)
        (*result)[jj] /= activation;
    }
    else
      throw Exception("Unknown projection for LinearRepresentation");
  }
  
  return (*result)[0];
}

void LinearRepresentation::write(const ProjectionPtr projection, const Vector &target, double alpha)
{
  // TODO: Store read values and update those (for thread safety)
  Vector value;
  read(projection, &value);
  Vector delta = alpha*(target-value);
  
  update(projection, delta);
}

void LinearRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  Projection &p = *projection;
  
  IndexProjection *ip = dynamic_cast<IndexProjection*>(&p);
  if (ip)
  {
    for (size_t ii=0; ii != ip->indices.size(); ++ii)
      for (size_t jj=0; jj < outputs_; ++jj)
        if (ip->indices[ii] != IndexProjection::invalid_index())
          params_[ip->indices[ii]*outputs_+jj] += delta[jj];
  }
  else
  {
    VectorProjection *vp = dynamic_cast<VectorProjection*>(&p);
    if (vp)
    {
      for (size_t ii=0; ii != vp->vector.size(); ++ii)
        for (size_t jj=0; jj < outputs_; ++jj)
          params_[ii*outputs_+jj] += vp->vector[ii]*delta[jj];
    }
    else
      throw Exception("Unknown projection for LinearRepresentation");
  }
}
