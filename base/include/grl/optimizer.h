/*
 * optimizer.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include <grl/configurable.h>
#include <grl/policy.h>

namespace grl
{

/// Optimizes a Policy from rollouts.
class Optimizer : public Configurable
{
  public:
    virtual ~Optimizer() { }
    virtual Optimizer *clone() const = 0;
    virtual size_t size() const = 0;
    virtual Policy *request(size_t ii) const = 0;
    virtual void report(size_t ii, double reward) = 0;
};

}

#endif /* OPTIMIZER_H_ */
