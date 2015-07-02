/** \file bounded_q.h
 * \brief Bounded Q policy header file.
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

#ifndef GRL_BOUNDED_Q_POLICY_H_
#define GRL_BOUNDED_Q_POLICY_H_

#include <grl/policies/q.h>

namespace grl
{

/// Q Policy with bounded action deltas.
class BoundedQPolicy : public QPolicy
{
  public:
    TYPEINFO("policy/discrete/q/bounded", "Q-value based policy with bounded action deltas")

  protected:
    Vector bound_;
    Vector prev_out_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From QPolicy
    virtual BoundedQPolicy *clone() const;
    virtual void act(double time, const Vector &in, Vector *out);
    
  protected:
    /// Filter out actions that lie outside bounds.
    void filter(const Vector &prev_out, const Vector &qvalues, Vector *filtered, std::vector<size_t> *idx) const;
};

}

#endif /* GRL_BOUNDED_Q_POLICY_H_ */
