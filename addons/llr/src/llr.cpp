#include <grl/representations/llr.h>
#include <grl/projections/sample.h>

using namespace grl;

REGISTER_CONFIGURABLE(LLRRepresentation)

void LLRRepresentation::request(ConfigurationRequest *config)
{
}

void LLRRepresentation::configure(Configuration &config)
{
  projector_ = (SampleProjector*)config["projector"].ptr();
  ridge_regression_factor_ = config["ridge"];
  outputs_ = 0;
}

void LLRRepresentation::reconfigure(const Configuration &config)
{
}

LLRRepresentation *LLRRepresentation::clone() const
{
  return NULL;
}

double LLRRepresentation::read(const ProjectionPtr &projection, Vector *result) const
{
  Matrix A, b;

  SampleProjection *p = dynamic_cast<SampleProjection*>(projection.get());
  grl_assert(p);

  // Convert query
  RowVector q(p->query.size()+1);
  
  for (size_t ii=0; ii < p->query.size(); ++ii)
    q[ii] = p->query[ii];
  q[p->query.size()] = 1.;

  // Fill matrices A and b
  {
    Guard guard(*p->store);
    for (size_t ii=0; ii < p->samples.size(); ++ii)
    {
      for (size_t jj=0; jj < p->query.size(); ++jj)
        A(ii, jj) = (*p->store)[p->samples[ii]]->in[jj]*p->weights[ii];
      A(ii, p->query.size()) = p->weights[ii];
      
      for (size_t jj=0; jj < outputs_; ++jj)
        b(ii, jj) = (*p->store)[p->samples[ii]]->out[jj]*p->weights[ii];
    }
  }

  // Solve with ATA*x = ATb with QR
  Matrix At = A.transpose();  
  Matrix ATAL = At*A + Matrix::Identity(A.cols(), A.cols())*ridge_regression_factor_;
  Eigen::LLT<Matrix> decompATAL(ATAL);
  if (decompATAL.info() != Eigen::Success)
    return false;

  Matrix r = decompATAL.solve(At);
  if (decompATAL.info() != Eigen::Success)
    return false;

  RowVector y = q*(r*b);
 
  return y[0];
}

void LLRRepresentation::write(const ProjectionPtr projection, const Vector &target, double alpha)
{
  SampleProjection *p = dynamic_cast<SampleProjection*>(projection.get());
  grl_assert(p);
  
  grl_assert(!outputs_ || target.size() == outputs_);
  outputs_ = target.size();

  // Push query on store
  Sample *sample = new Sample();

  for (size_t ii=0; ii < p->query.size(); ++ii)
    sample->in[ii] = p->query[ii];
    
  for (size_t ii=0; ii < target.size(); ++ii)
    sample->out[ii] = target[ii];
    
  sample->relevance = 1.;
  
  projector_->push(sample);
}

void LLRRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
}
                