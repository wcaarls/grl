/*
 * random.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef GRL_RANDOM_POLICY_H_
#define GRL_RANDOM_POLICY_H_

#include <grl/policy.h>
#include <grl/discretizer.h>

namespace grl
{

/// Random policy
class RandomPolicy : public Policy
{
  public:
    TYPEINFO("policy/random")

  protected:
    Vector min_, max_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual RandomPolicy *clone() const;
    virtual void act(const Vector &in, Vector *out) const;
};

/// Discrete random policy
class RandomDiscretePolicy : public Policy
{
  public:
    TYPEINFO("policy/discrete/random")

  protected:
    Discretizer *discretizer_;
    std::vector<Vector> options_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From DiscretePolicy
    virtual RandomDiscretePolicy *clone() const;
    virtual void act(const Vector &in, Vector *out) const;
    virtual void distribution(const Vector &in, Vector *out) const;
};

}

#endif /* GRL_RANDOM_POLICY_H_ */
