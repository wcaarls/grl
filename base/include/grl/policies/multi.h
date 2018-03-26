/** \file multi.h
 * \brief Policy combiner header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-03-13
 *
 * \copyright \verbatim
 * Copyright (c) 2018, Wouter Caarls
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

#ifndef GRL_MULTI_POLICY_H_
#define GRL_MULTI_POLICY_H_

#include <grl/policy.h>
#include <grl/discretizer.h>

namespace grl
{

/// Policy that combines two or more sub-policies using different strategies
class MultiPolicy : public Policy
{
  public:
    TYPEINFO("mapping/policy/multi", "Combines multiple policies")
    
    enum CombinationStrategy {csAddProbabilities, csMultiplyProbabilities, csMajorityVotingProbabilities, csRankVotingProbabilities};

  protected:
    std::string strategy_str_;
    CombinationStrategy strategy_;
    Discretizer *discretizer_;
    TypedConfigurableList<Policy> policy_;

  public:
    MultiPolicy() { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual void act(const Observation &in, Action *out) const;
    virtual void act(double time, const Observation &in, Action *out);
    virtual void distribution(const Observation &in, const Action &prev, LargeVector *out) const;
};

}

#endif /* GRL_MULTI_POLICY_H_ */
