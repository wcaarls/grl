/*
 * action_policy.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef ACTION_POLICY_H_
#define ACTION_POLICY_H_

#include <grl/policy.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/discretizer.h>

namespace grl
{

/// Policy based on an rqAction Representation.
class DeterministicActionPolicy : public Policy
{
  public:
    TYPEINFO("policy/action/deterministic")

  protected:
    Projector *projector_;
    Representation *representation_;

  public:
    virtual Policy *clone() const;
    virtual void act(const Vector &in, Vector *out) const;
};

/// Policy based on an rqProbability Representation.
class StochasticActionPolicy : public Policy
{
  public:
    TYPEINFO("policy/action/stochastic")

  protected:
    Discretizer *discretizer_;
    Projector *projector_;
    Representation *representation_;

  public:
    virtual Policy *clone() const;
    virtual void act(const Vector &in, Vector *out) const;
};

}

#endif /* ACTION_POLICY_H_ */
