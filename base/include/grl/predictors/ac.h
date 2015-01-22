/** \file ac.h
 * \brief Actor-critic predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
 *
 * \copyright \verbatim
 * Copyright (c) 2015, Wouter Caarls
 * All rights reserved.
 *
 * This file is part of GRL, the Generic Reinforcement Learning library.
 *
 * GRL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * \endverbatim
 */

#ifndef GRL_AC_PREDICTOR_H_
#define GRL_AC_PREDICTOR_H_

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

#endif /* GRL_AC_PREDICTOR_H_ */
