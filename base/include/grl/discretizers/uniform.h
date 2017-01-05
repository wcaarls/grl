/** \file uniform.h
 * \brief Uniform discretizer header file.
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

#ifndef GRL_UNIFORM_DISCRETIZER_H_
#define GRL_UNIFORM_DISCRETIZER_H_

#include <grl/configurable.h>
#include <grl/discretizer.h>

namespace grl
{

/// Uniform discretization
class UniformDiscretizer : public Discretizer
{
  public:
    TYPEINFO("discretizer/uniform", "Uniform discretizer")

  protected:
    Vector min_, max_, steps_;
  
    std::vector<LargeVector> values_;

  public:
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Discretizer
    virtual UniformDiscretizer* clone();
    virtual iterator begin() const;
    virtual size_t size() const;
    virtual void inc(iterator *it) const;
    virtual Vector get(const iterator &it) const;
    virtual Vector at(size_t idx) const;
    virtual size_t discretize(Vector *vec) const;
};

}

#endif /* GRL_UNIFORM_DISCRETIZER_H_ */
