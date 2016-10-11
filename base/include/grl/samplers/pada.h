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
#include <grl/samplers/greedy.h>
#include <grl/signals/signal_v.h>

namespace grl
{

/// Maximum search with a PADA random chance of non-maximums.
/// For details see "Learning while preventing mechanical failure due to random motions"
/// by H. J. Meijdam, M. C. Plooij and W. Caarls
class PADASampler : public EpsilonGreedySampler
{
  public:
    TYPEINFO("sampler/pada", "Maximum search with a PADA random chance of non-maximums")

  protected:
    Discretizer *discretizer_;
    mutable std::vector<size_t> sample_idx_;
    Vector steps_;
    Vector delta_;
    Signal *mirror_sig_;

  public:
    PADASampler() { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Sampler
    virtual PADASampler *clone();
    virtual size_t sample(const Vector &values, TransitionType &tt) const;

  protected:
    void increment(std::vector<size_t> &idx, const std::vector<size_t> &lower_idx, const std::vector<size_t> &upper_idx) const;
};

}

#endif /* GRL_PADA_SAMPLER_H_ */
