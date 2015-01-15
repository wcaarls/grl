#include <grl/policies/q.h>

using namespace grl;

REGISTER_CONFIGURABLE(QPolicy)

void QPolicy::request(ConfigurationRequest *config)
{
}

void QPolicy::configure(const Configuration &config)
{
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  discretizer_->options(&variants_);
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  sampler_ = (Sampler*)config["sampler"].ptr();
}

void QPolicy::reconfigure(const Configuration &config)
{
}

QPolicy *QPolicy::clone() const
{
  QPolicy *qp = new QPolicy();
  qp->discretizer_ = discretizer_->clone();
  qp->projector_ = projector_->clone();
  qp->representation_ = representation_->clone();
  qp->sampler_ = sampler_->clone();
  
  return qp;
}

void QPolicy::values(const Vector &in, Vector *out) const
{
  std::vector<ProjectionPtr> projections;
  projector_->project(in, variants_, &projections);
  
  out->resize(variants_.size());
  Vector value;
  for (size_t ii=0; ii < variants_.size(); ++ii)
  {
    representation_->read(projections[ii], &value);
    (*out)[ii] = value[0];
  }
}

void QPolicy::act(const Vector &in, Vector *out) const
{
  Vector qvalues;
  
  values(in, &qvalues);
  size_t action = sampler_->sample(qvalues);
  
  *out = variants_[action];
}
            