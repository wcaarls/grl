/** \file fqi.h
 * \brief Fitted Q-iteration predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-04-28
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

#ifndef GRL_FQI_PREDICTOR_H_
#define GRL_FQI_PREDICTOR_H_

#include <grl/predictor.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/policies/q.h>

namespace grl
{

/// Fitted Q-iteration predictor.
class FQIPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/fqi", "Fitted Q-iteration predictor")

  protected:
    double gamma_;
    Projector *projector_;
    Representation *representation_;
    QPolicy *policy_;
    size_t max_samples_, iterations_;
    std::vector<Transition> transitions_; // Transitions of all episodes are saved here
    size_t rebuild_counter_, rebuild_batch_size_;

  public:
    FQIPredictor() : gamma_(0.97), projector_(NULL), representation_(NULL), policy_(NULL),
      max_samples_(100000), iterations_(10), rebuild_counter_(1), rebuild_batch_size_(10) { }
    void rebuild();

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual void update(const Transition &transition);
    virtual void finalize();
    
    // From BatchPredictor
    virtual FQIPredictor *clone() const;
};

}

#endif /* GRL_FQI_PREDICTOR_H_ */
