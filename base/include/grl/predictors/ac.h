/*
 * ac_predictor.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef AC_PREDICTOR_H_
#define AC_PREDICTOR_H_

#include <grl/representation.h>
#include <grl/predictor.h>

namespace grl
{

/// Actor-critic predictor for \link DeterministicActionPolicy DeterministicActionPolicies \endlink.
class DeterministicACPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/ac/deterministic")

  protected:
    Representation *critic_;
    Representation *actor_;
};

/// Actor-critic predictor for \link StochasticActionPolicy StochasticActionPolicies \endlink.
class StochasticACPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/ac/stochastic")
    
  protected:
    Representation *critic_;
    Representation *actor_;
};

}

#endif /* AC_PREDICTOR_H_ */
