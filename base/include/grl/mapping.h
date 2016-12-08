/** \file mapping.h
 * \brief Generic mapping definitions.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-06-01
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

#ifndef GRL_MAPPING_H_
#define GRL_MAPPING_H_

#include <grl/configurable.h>
#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Maps inputs to outputs.
class Mapping : public Configurable
{
  public:
    /**
     * \brief Read out the mapping.
     *
     * Returns the value of the first output dimension.
     */
    virtual double read(const Vector &in, Vector *result) const = 0;
};

class RepresentedMapping : public Mapping
{
  public:
    TYPEINFO("mapping/represented", "A mapping that internally uses a representation")

  protected:
    Projector *projector_;
    Representation *representation_;
  
  public:
    RepresentedMapping() : projector_(NULL), representation_(NULL)
    {
    }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config)
    {
      config->push_back(CRP("projector", "projector", "Projects inputs onto representation space", projector_));
      config->push_back(CRP("representation", "representation", "Representation", representation_));
    }
    
    virtual void configure(Configuration &config)
    {
      projector_ = (Projector*)config["projector"].ptr();
      representation_ = (Representation*)config["representation"].ptr();
    }
    
    virtual void reconfigure(const Configuration &config)
    {
    }

    // From Mapping
    virtual double read(const Vector &in, Vector *result) const
    {
      return representation_->read(projector_->project(in), result);
    }
};

}

#endif /* GRL_MAPPING_H_ */
