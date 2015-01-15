/*
 * policy.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef POLICY_H_
#define POLICY_H_

#include <grl/configurable.h>

namespace grl
{

/// Maps states to actions.
class Policy : public Configurable
{
  public:
    virtual ~Policy() { }
    virtual Policy *clone() const = 0;
    virtual void act(const Vector &in, Vector *out) const = 0;
};

}

#endif /* POLICY_H_ */
