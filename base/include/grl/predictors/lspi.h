/** \file lspi.h
 * \brief Least Squares Policy Iteration predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-04-30
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

#ifndef GRL_LSPI_PREDICTOR_H_
#define GRL_LSPI_PREDICTOR_H_

#include <Eigen/Sparse>

#include <grl/predictor.h>
#include <grl/discretizer.h>
#include <grl/projector.h>
#include <grl/representations/linear.h>

namespace grl
{

/// Fitted Q-iteration predictor.
class LSPIPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/lspi", "Least Squares Policy Iteration predictor")
    
    struct CachedTransition
    {
      Eigen::SparseVector<double> phi;
      Eigen::SparseMatrix<double> nextphi;
      
      std::vector<ProjectionPtr> actions;
      Transition transition;
      
      CachedTransition(const Transition &t = Transition()) : transition(t) { }
    };
    
    enum ResetStrategy { rsNever, rsBatch, rsIteration };

  protected:
    double gamma_;
    Discretizer *discretizer_;
    Projector *projector_;
    LinearRepresentation *representation_;
    
    std::vector<Vector> variants_;
    
    size_t max_samples_, iterations_;
    std::vector<CachedTransition> transitions_;
    size_t macro_batch_size_, macro_batch_counter_;

  public:
    LSPIPredictor() : gamma_(0.97), discretizer_(NULL), projector_(NULL), representation_(NULL), max_samples_(100000), iterations_(10), macro_batch_size_(1), macro_batch_counter_(0) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual LSPIPredictor &copy(const Configurable &obj);
    
    // From Predictor
    virtual void update(const Transition &transition);
    virtual void finalize();

  protected:
    virtual void rebuild();
};

}

#endif /* GRL_LSPI_PREDICTOR_H_ */
