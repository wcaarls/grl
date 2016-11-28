/** \file sampler.h
 * \brief Generic sampler definition.
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

#ifndef GRL_SAMPLER_H_
#define GRL_SAMPLER_H_

#include <grl/configurable.h>
#include <grl/utils.h>
#include <grl/grl.h>

namespace grl
{

/// Samples an action from a value Vector.
class Sampler : public Configurable
{
  public:
    virtual ~Sampler() { }
    virtual Sampler *clone() = 0;
    
    /// Sample an action from a value vector.
    virtual size_t sample(const LargeVector &values, TransitionType &tt) const = 0;
    
    /// Returns the sampling distribution for a value vector.
    virtual void distribution(const LargeVector &values, LargeVector *distribution) const = 0;
};

}

#endif /* GRL_SAMPLER_H_ */
