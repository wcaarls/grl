/** \file policy.h
 * \brief Policy discretizer header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-09-01
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#ifndef GRL_POLICY_DISCRETIZER_H_
#define GRL_POLICY_DISCRETIZER_H_

#include <grl/configurable.h>
#include <grl/discretizer.h>
#include <grl/policy.h>

namespace grl
{

/// Discretizer that returns the action suggested by a policy.
class PolicyDiscretizer : public Discretizer
{
  public:
    TYPEINFO("discretizer/policy", "Returns the action suggested by a policy")

  protected:
    Policy *policy_;

  public:
    PolicyDiscretizer() : policy_(NULL) { }
  
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Discretizer
    virtual PolicyDiscretizer* clone();
    virtual iterator begin(const Vector &point) const;
    virtual size_t size() const;
    virtual void inc(iterator *it) const;
    virtual Vector get(const iterator &it) const;
    virtual Vector at(const Vector &point, size_t idx) const;


    virtual Vector steps()  const { return Vector(); }

    virtual void discretize(Vector &vec, IndexVector *idx_v = NULL) const {}
    virtual size_t offset(const IndexVector &idx) const { return 0; }
};

}

#endif /* GRL_POLICY_DISCRETIZER_H_ */
