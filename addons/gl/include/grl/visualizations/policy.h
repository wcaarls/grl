/** \file policy.h
 * \brief Policy visualization header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-14
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

#ifndef GRL_POLICY_VISUALIZATION_H_
#define GRL_POLICY_VISUALIZATION_H_

#include <string.h>
#include <pthread.h>

#include <grl/policy.h>
#include <grl/visualizations/field.h>

namespace grl
{

/// Policy visualization.
class PolicyVisualization : public FieldVisualization
{
  public:
    TYPEINFO("visualization/field/policy/action", "Visualizes a policy over a field of states")

  protected:
    Policy *policy_;
    size_t dim_;
  
  public:
    PolicyVisualization() : policy_(NULL), dim_(0) { }
    ~PolicyVisualization()
    {
      stopAndJoin();
    }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From FieldVisualization
    virtual double value(const Vector &in) const;
};

}

#endif /* GRL_POLICY_VISUALIZATION_H_ */
