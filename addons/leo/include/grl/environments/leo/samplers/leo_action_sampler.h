/** \file leo_action_sampler.h
 * \brief Wrapper for an action sampler for Leo which supports contact signals (header file).
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-09-30
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

#ifndef GRL_LEO_ACTION_SAMPLER_H_
#define GRL_LEO_ACTION_SAMPLER_H_

#include <grl/sampler.h>
#include <grl/utils.h>
#include <grl/grl.h>
#include <grl/discretizer.h>
#include <grl/signal.h>

namespace grl
{

class LeoActionSampler : public Sampler
{
  public:
    TYPEINFO("sampler/leo/action", "Wrapper for an action sampler for Leo (can modify memory of samplers with memory at contact events)")

  protected:
    Sampler *sampler_;
    VectorSignal *sub_ic_signal_;
    VectorSignal *pub_sub_sampler_state_;

  public:
    LeoActionSampler() : sampler_(NULL), sub_ic_signal_(NULL), pub_sub_sampler_state_(NULL) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Sampler
    virtual size_t sample(double time, const LargeVector &values, TransitionType &tt);
    virtual void distribution(const LargeVector &values, LargeVector *distribution);
};

}

#endif /* GRL_LEO_ACTION_SAMPLER_H_ */
