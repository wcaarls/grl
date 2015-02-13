/** \file parameterized.h
 * \brief Continuous and discrete parameterized policies header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-13
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

#ifndef GRL_PARAMETERIZED_POLICY_H_
#define GRL_PARAMETERIZED_POLICY_H_

#include <grl/policy.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/discretizer.h>

namespace grl
{

/// Policy based on a direct action representation
class ContinuousParameterizedPolicy : public ParameterizedPolicy
{
  public:
    TYPEINFO("policy/parameterized/continuous")

  protected:
    Projector *projector_;
    ParameterizedRepresentation *representation_;
    
    Vector min_, max_;

  public:
    ContinuousParameterizedPolicy() : projector_(NULL), representation_(NULL) { }
    
    // From Configurable  
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual ContinuousParameterizedPolicy *clone() const;
    virtual void act(const Vector &in, Vector *out) const;
    
    // From ParameterizedPolicy
    virtual size_t size() const { return representation_->size(); }
    virtual const Vector &params() const { return representation_->params(); }
    virtual Vector &params() { return representation_->params(); }
    
};

/// Policy based on an action-probability representation.
class DiscreteParameterizedPolicy : public ParameterizedPolicy
{
  public:
    TYPEINFO("policy/parameterized/discrete")

  protected:
    Discretizer *discretizer_;
    Projector *projector_;
    ParameterizedRepresentation *representation_;
    
    std::vector<Vector> variants_;

  public:
    DiscreteParameterizedPolicy() : discretizer_(NULL), projector_(NULL), representation_(NULL) { }
  
    // From Configurable  
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Policy
    virtual DiscreteParameterizedPolicy *clone() const;
    virtual void act(const Vector &in, Vector *out) const;
    
    // From ParameterizedPolicy
    virtual size_t size() const { return representation_->size(); }
    virtual const Vector &params() const { return representation_->params(); }
    virtual Vector &params() { return representation_->params(); }
};

}

#endif /* GRL_PARAMETERIZED_POLICY_H_ */
