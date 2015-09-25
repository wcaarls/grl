/** \file predictor.h
 * \brief Generic predictor definition.
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

#ifndef GRL_PREDICTOR_H_
#define GRL_PREDICTOR_H_

#include <grl/grl.h>
#include <grl/configurable.h>
#include <grl/importer.h>
#include <grl/exporter.h>

namespace grl
{

/// Estimates a function from Transition%s.
class Predictor : public Configurable
{
  protected:
    Importer *importer_;
    Exporter *exporter_;

  public:
    Predictor() : importer_(NULL), exporter_(NULL) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    virtual Predictor *clone() const = 0;
    
    /// Update the estimation.
    virtual void update(const Transition &transition);
    
    /// Signal completion of a set of updates (episode or batch).
    virtual void finalize();
};

}

#endif /* GRL_PREDICTOR_H_ */
