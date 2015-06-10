/** \file greedy.h
 * \brief Greedy and Epsilon-greedy samplers header file.
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

#ifndef GRL_GREEDY_SAMPLER_H_
#define GRL_GREEDY_SAMPLER_H_

#include <grl/sampler.h>
#include <grl/utils.h>

namespace grl
{

/// Maximum search.
class GreedySampler : public Sampler
{
  public:
    TYPEINFO("sampler/greedy", "Maximum search")

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Sampler
    virtual GreedySampler *clone();
    virtual size_t sample(const Vector &values) const;
    virtual void distribution(const Vector &values, Vector *distribution) const;
};

/// Maximum search with a uniform random chance of non-maximums.
class EpsilonGreedySampler : public GreedySampler
{
  public:
    TYPEINFO("sampler/epsilon_greedy", "Maximum search with a uniform random chance of non-maximums")

  protected:
    double epsilon_;
    Rand *rand_;

  public:
    EpsilonGreedySampler() : epsilon_(0.05) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Sampler
    virtual EpsilonGreedySampler *clone();
    virtual size_t sample(const Vector &values) const;
    virtual void distribution(const Vector &values, Vector *distribution) const;
};

}

#endif /* GRL_GREEDY_SAMPLER_H_ */
