/** \file pada.h
 * \brief PADA samplers header file.
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

#ifndef GRL_PADA_SAMPLER_H_
#define GRL_PADA_SAMPLER_H_

#include <grl/sampler.h>
#include <grl/utils.h>
#include <grl/grl.h>
#include <grl/discretizer.h>
#include <grl/signal.h>
#include <grl/samplers/greedy.h>

namespace grl
{

/// Maximum search with a PADA random chance of non-maximums.
/// For details see "Learning while preventing mechanical failure due to random motions"
/// by H. J. Meijdam, M. C. Plooij and W. Caarls
class PadaSampler : public EpsilonGreedySampler
{
  public:
    TYPEINFO("sampler/pada", "Maximum search with a PADA random chance of non-maximums")

  protected:
    Discretizer *discretizer_;
    size_t offset_;
    Vector delta_;
    VectorSignal *sub_ic_signal_;

    VectorSignal *pub_sub_sampler_state_;

  public:
    PadaSampler() : offset_(0) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Sampler
    virtual PadaSampler *clone();
    virtual size_t sample(const LargeVector &values, TransitionType &tt);

  public:
    virtual void set_offset(size_t offset) { offset_ = offset; }

  protected:
    virtual Vector env_signal_processor();
    virtual void get_bounds(size_t offset, Vector &delta, IndexVector &lower_bound, IndexVector &upper_bound) const;
    virtual size_t exploration_step(Discretizer::bounded_iterator &bit) const;
    virtual size_t exploitation_step(const LargeVector &values, Discretizer::bounded_iterator &bit) const;
};


class EpsilonPadaSampler : public PadaSampler
{
  public:
    TYPEINFO("sampler/epsilon_pada", "exploitations are done by greedy action selection without constraints, as in e-greedy. Explorations are done with constrained set of actions, as it is in pada.")

  public:
    EpsilonPadaSampler() { }

    // From Sampler
    virtual EpsilonPadaSampler *clone();
    virtual size_t sample(const LargeVector &values, TransitionType &tt);
};

}

#endif /* GRL_PADA_SAMPLER_H_ */
