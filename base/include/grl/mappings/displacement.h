/** \file value.h
 * \brief Policy displacement mapping definition.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-13
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

#ifndef GRL_DISPLACEMENT_MAPPING_H_
#define GRL_DISPLACEMENT_MAPPING_H_

#include <grl/mapping.h>
#include <grl/policy.h>
#include <grl/environments/observation.h>

namespace grl
{

class DisplacementMapping : public Mapping
{
  public:
    TYPEINFO("mapping/displacement", "Mapping that returns the state displacement effected by a policy")

  protected:
    Policy *policy_;
    ObservationModel *model_;
      
  public:
    DisplacementMapping() : policy_(NULL), model_(NULL)
    {
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Mapping
    virtual double read(const Vector &in, Vector *result) const;
};

}

#endif /* GRL_DISPLACEMENT_MAPPING_H_ */
