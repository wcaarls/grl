/*
 * grl.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef GRL_H_
#define GRL_H_

#include <boost/shared_ptr.hpp>
#include <vector>
#include <assert.h>
#include <math.h>

#include <grl/vector.h>

namespace grl
{

/// Possible targets of a Representation.
enum RepresentedQuantity
{
  rqStateActionValue, rqStateValue, rqState, rqRewardTerminal, rqProbability
};

/// Basic (s, a, r, s', a') state transition.
struct Transition
{
  Vector prev_obs;
  Vector prev_action;
  double reward;
  Vector obs;
  Vector action;
  
  Transition(Vector _prev_obs, Vector _prev_action, double _reward, Vector _obs=Vector(), Vector _action=Vector()) :
    prev_obs(_prev_obs), prev_action(_prev_action), reward(_reward), obs(_obs), action(_action)
    {
    }
};

}

#endif /* GRL_H_ */
