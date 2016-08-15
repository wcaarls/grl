/** \file advised_action.h
 * \brief Advised Action policy header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-11
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

#ifndef GRL_ADVISED_ACTION_POLICY_H_
#define GRL_ADVISED_ACTION_POLICY_H_

#include <grl/policies/action.h>

namespace grl
{

/// Action Policy with advised action.
class AdvisedActionPolicy : public ActionPolicy
{
  public:
    TYPEINFO("policy/action/advised", "Action policy with an advised action")

  public:
    // From Configurable
    //virtual void request(ConfigurationRequest *config);
    //virtual void configure(Configuration &config);
    //virtual void reconfigure(const Configuration &config);

    // From QPolicy
    virtual AdvisedActionPolicy *clone() const;
    virtual TransitionType act(const Vector &in, Vector *out) const;
};

}

#endif /* GRL_ADVISED_ACTION_POLICY_H_ */
