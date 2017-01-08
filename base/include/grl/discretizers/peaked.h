/** \file peaked.h
 * \brief Peaked discretizer header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-09-01
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

#ifndef GRL_PEAKED_DISCRETIZER_H_
#define GRL_PEAKED_DISCRETIZER_H_

#include <grl/configurable.h>
#include <grl/discretizers/uniform.h>

namespace grl
{

/// Peaked discretization
class PeakedDiscretizer : public UniformDiscretizer
{
  public:
    TYPEINFO("discretizer/peaked", "Peaked discretizer, with more resolution around center")

  protected:
    Vector peaking_;

  public:
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
};

}

#endif /* GRL_PEAKED_DISCRETIZER_H_ */
