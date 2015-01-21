#include <grl/representations/multisine.h>

using namespace grl;

REGISTER_CONFIGURABLE(MultisineMapping)

void MultisineMapping::request(ConfigurationRequest *config)
{
}

void MultisineMapping::configure(Configuration &config)
{
  config.get("outputs", outputs_, (size_t)1);
  config.get("sines", sines_, (size_t)1);
  config.get("inputs", inputs_, (size_t)1);

  params_ = RandGen::getVector(p(outputs_, 0, 0, 0));

  // Choose some sensible scales  
  for (size_t oo=0; oo < outputs_; ++oo)
    for (size_t ss=0; ss < sines_; ++ss)
      for (size_t ii=0; ii < inputs_; ++ii)
        params_[p(oo, ss, ii, 1)] = M_PI+3*M_PI*params_[p(oo, ss, ii, 1)];
}

void MultisineMapping::reconfigure(const Configuration &config)
{
}

MultisineMapping *MultisineMapping::clone() const
{
  return new MultisineMapping(*this);
}

double MultisineMapping::read(const ProjectionPtr &projection, Vector *result) const
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  grl_assert(vp);

  result->resize(outputs_);
  
  for (size_t oo=0; oo < outputs_; ++oo)
  {
    double r = 0;
    
    for (size_t ss=0; ss < sines_; ++ss)
    {
      // Amplitude
      double rs = params_[p(oo, ss+1, 0, -1)];
      
      // Multiple by activation over each input
      for (size_t ii=0; ii < inputs_; ++ii)
        rs *= sin(params_[p(oo, ss, ii, 0)] + params_[p(oo, ss, ii, 1)]*vp->vector[ii]);

      // Add all sines together
      r += rs;
    }
        
    (*result)[oo] = r;
  }
  
  return (*result)[0];
}
