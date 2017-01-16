/** \file puddle.h
 * \brief Puddle mapping header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-10
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

#ifndef GRL_PUDDLE_MAPPING_H_
#define GRL_PUDDLE_MAPPING_H_

#include <grl/mapping.h>

namespace grl
{

/// Sum of sines, used for testing.
class PuddleMapping : public Mapping
{
  public:
    TYPEINFO("mapping/puddle", "Random 2D puddles")
    
  protected:
    size_t seed_;
    double smoothing_, steepness_;
    Matrix map_;

  public:
    PuddleMapping() : seed_(1), smoothing_(0.1), steepness_(1.), map_(65, 65) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Mapping
    virtual double read(const Vector &in, Vector *result) const;
};

}

#endif /* GRL_PUDDLE_MAPPING_H_ */
