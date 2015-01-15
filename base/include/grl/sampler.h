/*
 * sampler.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef SAMPLER_H_
#define SAMPLER_H_

#include <grl/configurable.h>
#include <grl/utils.h>

namespace grl
{

/// Samples an action from a value Vector.
class Sampler : public Configurable
{
  public:
    virtual ~Sampler() { }
    virtual Sampler *clone() = 0;
    virtual size_t sample(const Vector &values) const = 0;
    virtual void distribution(const Vector &values, Vector *distribution) const = 0;
};

}

#endif /* SAMPLER_H_ */
