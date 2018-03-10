/** \file image.h
 * \brief Image file mapping definition.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-03-08
 *
 * \copyright \verbatim
 * Copyright (c) 2018, Wouter Caarls
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

#ifndef GRL_IMAGE_MAPPING_H_
#define GRL_IMAGE_MAPPING_H_

#include <grl/mapping.h>
#include <grl/importer.h>

namespace grl
{

/// 
class ImageMapping : public Mapping
{
  public:
    TYPEINFO("mapping/image", "Mapping read from an ICS image file")

  protected:
    std::string file_;
    Vector min_, max_;
    double *data_;
    IndexVector dims_;
  
  public:
    ImageMapping() : data_(NULL)
    {
      min_ = ConstantVector(2, 0.);
      max_ = ConstantVector(2, 1.);
    }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Mapping
    virtual double read(const Vector &in, Vector *result) const;
};

}

#endif /* GRL_IMAGE_MAPPING_H_ */
