#include <grl/policies/random.h>

using namespace grl;

REGISTER_CONFIGURABLE(RandomPolicy)
REGISTER_CONFIGURABLE(RandomDiscretePolicy)

void RandomPolicy::request(ConfigurationRequest *config)
{
}

void RandomPolicy::configure(Configuration &config)
{
  min_ = config["min"];
  max_ = config["max"];
  
  grl_assert(min_.size() == max_.size());
}

void RandomPolicy::reconfigure(const Configuration &config)
{
}

RandomPolicy *RandomPolicy::clone() const
{
  return new RandomPolicy(*this);
}

void RandomPolicy::act(const Vector &in, Vector *out) const
{
  out->resize(min_.size());
  Rand *rand = RandGen::instance();
  
  for (size_t ii=0; ii < min_.size(); ++ii)
    (*out)[ii] = rand->getUniform(min_[ii], max_[ii]);
}

void RandomDiscretePolicy::request(ConfigurationRequest *config)
{
}

void RandomDiscretePolicy::configure(Configuration &config)
{
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  discretizer_->options(&options_);
}

void RandomDiscretePolicy::reconfigure(const Configuration &config)
{
}

RandomDiscretePolicy *RandomDiscretePolicy::clone() const
{
  return new RandomDiscretePolicy(*this);
}

void RandomDiscretePolicy::act(const Vector &in, Vector *out) const
{
  *out = options_[RandGen::getInteger(options_.size())];
}

void RandomDiscretePolicy::distribution(const Vector &in, Vector *out) const
{
  out->resize(options_.size(), 1./options_.size());
}
