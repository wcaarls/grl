/*
 * q_policy.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef Q_POLICY_H_
#define Q_POLICY_H_

#include <grl/policy.h>
#include <grl/discretizer.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/sampler.h>

namespace grl
{

/// Policy based on an rqStateActionValue Representation.
class QPolicy : public Policy
{
  public:
    TYPEINFO("policy/q")

  protected:
    Discretizer *discretizer_;
    Projector *projector_;
    Representation *representation_;
    Sampler *sampler_;
    
    std::vector<Vector> variants_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual QPolicy *clone() const;
    virtual void values(const Vector &in, Vector *out) const;
    virtual void act(const Vector &in, Vector *out) const;
};

}

#endif /* Q_POLICY_H_ */
