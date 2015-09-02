/** \file state_feedback.h
 * \brief State feedback policy header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-27
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

#ifndef GRL_STATE_FEEDBACK_POLICY_H_
#define GRL_STATE_FEEDBACK_POLICY_H_

#include <grl/policies/parameterized.h>

namespace grl
{

/// State feedback policy
class StateFeedbackPolicy : public ParameterizedPolicy
{
  public:
    TYPEINFO("policy/parameterized/state_feedback", "Parameterized policy based on a state feedback controller")

  protected:
    Vector operating_state_, operating_action_;
    Vector gains_;
    Vector min_, max_;

  public:
    StateFeedbackPolicy() { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual StateFeedbackPolicy *clone() const;
    virtual void act(const Vector &in, Vector *out) const;
    
    // From ParameterizedPolicy
    virtual size_t size() const { return gains_.size(); }
    virtual const Vector &params() const { return gains_; }
    virtual Vector &params() { return gains_; }
};

}

#endif /* GRL_STATE_FEEDBACK_POLICY_H_ */
