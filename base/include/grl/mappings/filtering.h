/** \file filtering.h
 * \brief Filtering mapping definition.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-06-26
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
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

#ifndef GRL_FILTERING_MAPPING_H_
#define GRL_FILTERING_MAPPING_H_

#include <grl/mapping.h>
#include <grl/policy.h>
#include <grl/environments/observation.h>

namespace grl
{

class FilteringMapping : public Mapping
{
  public:
    TYPEINFO("mapping/filtering", "Mapping that filters inputs and outputs")

  protected:
    Mapping *mapping_;
    Vector input_idx_, output_idx_;
      
  public:
    FilteringMapping() : mapping_(NULL)
    {
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Mapping
    virtual double read(const Vector &in, Vector *result) const;
    virtual void read(const Matrix &in, Matrix *result) const;
};

class FilteringPolicy : public Policy
{
  public:
    TYPEINFO("mapping/policy/filtering", "Policy that filters observations and actions")
    
  protected:
    Policy *policy_;
    Vector observation_idx_, action_idx_;
    
  public:
    FilteringPolicy() : policy_(NULL)
    {
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual void act(double time, const Observation &in, Action *out);
    virtual void act(const Observation &in, Action *out) const;
};

}

#endif /* GRL_FILTERING_MAPPING_H_ */
