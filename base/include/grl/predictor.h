/*
 * predictor.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef PREDICTOR_H_
#define PREDICTOR_H_

#include <grl/grl.h>
#include <grl/configurable.h>

namespace grl
{

/// Estimates a value function from Transition%s.
class Predictor : public Configurable
{
  public:
    virtual ~Predictor() { }
    virtual Predictor *clone() const = 0;
    virtual void update(const Transition &transition) = 0;
    virtual void finalize() = 0;
};

/// Estimates a value function from batches of Transition%s.
class BatchPredictor : public Predictor
{
  public:
    virtual Predictor *clone() const = 0;
};

}

#endif /* PREDICTOR_H_ */
