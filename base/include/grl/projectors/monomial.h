/** \file monomial.h
 * \brief Monomial basis function projector header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-11-24
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

#ifndef GRL_MONOMIAL_PROJECTOR_H_
#define GRL_MONOMIAL_PROJECTOR_H_

#include <grl/projector.h>

namespace grl
{

/// Projects the input onto a set of monomial basis functions.
class MonomialProjector : public Projector
{
  public:
    TYPEINFO("projector/monomial", "Monomial basis function projector")
    
  protected:
    size_t inputs_, degree_, memory_;
    Vector operating_input_;

  public:
    MonomialProjector() : inputs_(1), degree_(1), memory_(0) { }

    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Projector
    virtual ProjectionLifetime lifetime() const { return plIndefinite; }
    virtual ProjectionPtr project(const Vector &in) const;
    
  protected:
    size_t fact(size_t n)
    {
      size_t f=1;
      for (size_t ii=2; ii <= n; ++ii)
        f *= ii;
        
      return f;
    }
};

}

#endif /* GRL_MONOMIAL_PROJECTOR_H_ */
