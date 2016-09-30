/** \file ornstein_uhlenbeck.h
 * \brief Ornstein-Uhlenbeck samplers header file.
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

#ifndef GRL_ORNSTEIN_UHLENBECK_SAMPLER_H_
#define GRL_ORNSTEIN_UHLENBECK_SAMPLER_H_

#include <grl/sampler.h>
#include <grl/utils.h>
#include <grl/grl.h>
#include <grl/discretizer.h>
#include <grl/samplers/greedy.h>

namespace grl
{

/// Maximum search with an Ornstein-Uhlenbeck random chance of non-maximums.
class OrnsteinUhlenbeckSampler : public EpsilonGreedySampler
{
  public:
    TYPEINFO("sampler/ornstein_ohlenbeck", "Maximum search with an Ornstein-Uhlenbeck random chance of non-maximums")

  protected:
    Discretizer *discretizer_;
    std::vector<Vector> variants_;
    Vector theta_, sigma_, center_;
    mutable size_t mai_;

  public:
    OrnsteinUhlenbeckSampler() { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Sampler
    virtual OrnsteinUhlenbeckSampler *clone();
    virtual size_t sample(const Vector &values, TransitionType &tt) const;
};

}

#endif /* GRL_ORNSTEIN_UHLENBECK_SAMPLER_H_ */
