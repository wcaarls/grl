/*
 * discretizer.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef DISCRETIZER_H_
#define DISCRETIZER_H_

#include <grl/configurable.h>

namespace grl
{

/// Provides a list of discrete points spanning a continuous space.
class Discretizer : public Configurable
{
  public:
    virtual ~Discretizer() { }
    virtual Discretizer* clone() = 0;
    virtual void options(std::vector<Vector> *out) const = 0;
};

}

#endif /* DISCRETIZER_H_ */
