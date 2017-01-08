/** \file multisine.h
 * \brief Multisine mapping header file.
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

#ifndef GRL_MULTISINE_MAPPING_H_
#define GRL_MULTISINE_MAPPING_H_

#include <grl/mapping.h>

namespace grl
{

/// Sum of sines, used for testing.
class MultisineMapping : public Mapping
{
  public:
    TYPEINFO("mapping/multisine", "Sum of sines mapping")
    
  protected:
    size_t outputs_, sines_, inputs_;
    
    Vector params_;

  public:
    MultisineMapping() : outputs_(1), sines_(1), inputs_(1) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Mapping
    virtual double read(const Vector &in, Vector *result) const ;
    
  protected:
    inline size_t p(size_t oo, size_t ss, size_t ii, size_t pp) const
    {
      return oo*(sines_*(1 + 2*inputs_)) + ss*(1 + 2*inputs_) + ii*2 + pp;
    }
};

}

#endif /* GRL_MULTISINE_MAPPING_H_ */
