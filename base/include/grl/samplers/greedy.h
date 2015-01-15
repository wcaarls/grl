/*
 * sampler.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef GREEDY_SAMPLER_H_
#define GREEDY_SAMPLER_H_

#include <grl/sampler.h>
#include <grl/utils.h>

namespace grl
{

/// Maximum search.
class GreedySampler : public Sampler
{
  public:
    TYPEINFO("sampler/greedy")

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(const Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Sampler
    virtual GreedySampler *clone();
    virtual size_t sample(const Vector &values) const;
    virtual void distribution(const Vector &values, Vector *distribution) const;
};

/// Maximum search with a uniform random chance of non-maximums.
class EpsilonGreedySampler : public GreedySampler
{
  public:
    TYPEINFO("sampler/epsilon_greedy")

  protected:
    double epsilon_;
    Rand *rand_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(const Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Sampler
    virtual EpsilonGreedySampler *clone();
    virtual size_t sample(const Vector &values) const;
    virtual void distribution(const Vector &values, Vector *distribution) const;
};

}

#endif /* GREEDY_SAMPLER_H_ */
