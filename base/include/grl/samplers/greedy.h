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

  protected:
    Rand *rand_;
    int rand_max_;

  public:
    GreedySampler() : rand_(NULL), rand_max_(0) { }
    ~GreedySampler() { if (rand_) { delete rand_; rand_ = NULL;} }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Sampler
    virtual size_t sample(const LargeVector &values, ActionType *at=NULL, double *logp=NULL) const;
    virtual void distribution(const LargeVector &values, LargeVector *distribution) const;
    
  protected:
    void findmax(const LargeVector &values, size_t &mai, size_t &man) const;
};

/// Maximum search with a uniform random chance of non-maximums.
class EpsilonGreedySampler : public GreedySampler
{
  public:
    TYPEINFO("sampler/epsilon_greedy", "Maximum search with a uniform random chance of non-maximums")

  protected:
    LargeVector epsilon_, distribution_;
    double distribution_sum_, decay_, decay_rate_, decay_min_;

  public:
    EpsilonGreedySampler() : distribution_sum_(0), decay_(1), decay_rate_(1), decay_min_(0)
    {
      epsilon_ = VectorConstructor(0.05);
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Sampler
    virtual size_t sample(double time, const LargeVector &values, ActionType *at=NULL, double *logp=NULL);
    virtual size_t sample(const LargeVector &values, ActionType *at=NULL, double *logp=NULL) const;
    virtual void distribution(const LargeVector &values, LargeVector *distribution) const;
    
  protected:
    LargeVector calculateBaseDistribution(const LargeVector &epsilon) const;
};

}

#endif /* GRL_GREEDY_SAMPLER_H_ */
