/** \file value.h
 * \brief Value function visualization header file.
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

#ifndef GRL_VALUE_VISUALIZATION_H_
#define GRL_VALUE_VISUALIZATION_H_

#include <string.h>
#include <pthread.h>

#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/policies/q.h>
#include <grl/visualizations/field.h>

namespace grl
{

/// Value function visualization.
class ValueVisualization : public FieldVisualization
{
  public:
    TYPEINFO("visualization/field/value", "Visualizes an approximation over a field of states")

  protected:
    Projector *projector_;
    Representation *representation_;
    size_t dim_;
  
  public:
    ValueVisualization() : projector_(NULL), representation_(NULL), dim_(0) { }
    ~ValueVisualization()
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

/// Policy-value function visualization.
class PolicyValueVisualization : public FieldVisualization
{
  public:
    TYPEINFO("visualization/field/policy/value", "Visualizes the value of a policy over a field of states")
    
  protected:
    Projector *projector_;
    Representation *representation_;
    QPolicy *policy_;
  
  public:
    PolicyValueVisualization() : projector_(NULL), representation_(NULL), policy_(NULL) { }
    ~PolicyValueVisualization()
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

#endif /* GRL_VALUE_VISUALIZATION_H_ */
