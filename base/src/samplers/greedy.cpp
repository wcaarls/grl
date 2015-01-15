#include <grl/samplers/greedy.h>

using namespace grl;

REGISTER_CONFIGURABLE(GreedySampler)
REGISTER_CONFIGURABLE(EpsilonGreedySampler)

void GreedySampler::request(ConfigurationRequest *config)
{
}

void GreedySampler::configure(const Configuration &config)
{
}

void GreedySampler::reconfigure(const Configuration &config)
{
}

GreedySampler *GreedySampler::clone()
{
  return new GreedySampler();
}

size_t GreedySampler::sample(const Vector &values) const
{
  size_t mai = 0;

  for (size_t ii=0; ii < values.size(); ++ii)
  {
    if (values[ii] > values[mai])
      mai = ii;
  }

  return mai;
}

void GreedySampler::distribution(const Vector &values, Vector *distribution) const
{
  size_t mai = sample(values);
  distribution->resize(values.size());

  for (size_t ii=0; ii < values.size(); ++ii)
  {
    if (ii == mai)
      (*distribution)[ii] = 1;
    else
      (*distribution)[ii] = 0;
  }
}

void EpsilonGreedySampler::request(ConfigurationRequest *config)
{
}

void EpsilonGreedySampler::configure(const Configuration &config)
{
  epsilon_ = config["epsilon"];
  rand_ = new Rand();
}

void EpsilonGreedySampler::reconfigure(const Configuration &config)
{
}

EpsilonGreedySampler *EpsilonGreedySampler::clone()
{
  EpsilonGreedySampler *egs = new EpsilonGreedySampler();
  egs->epsilon_ = epsilon_;
  egs->rand_ = rand_->clone();
  
  return egs;
}

size_t EpsilonGreedySampler::sample(const Vector &values) const
{
  if (rand_->get() < epsilon_)
    return rand_->getInteger(values.size());

  return GreedySampler::sample(values);
}

void EpsilonGreedySampler::distribution(const Vector &values, Vector *distribution) const
{
  GreedySampler::distribution(values, distribution);

  for (size_t ii=0; ii < values.size(); ++ii)
  {
    if ((*distribution)[ii] == 1)
      (*distribution)[ii] = 1-epsilon_;
    (*distribution)[ii] += epsilon_/values.size();
  }
}
