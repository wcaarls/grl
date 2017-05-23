/** \file lspi.cpp
 * \brief Least Squares Policy Iteration predictor source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-04-30
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

#include <eigen3/Eigen/SparseQR>

#include <grl/predictors/lspi.h>

using namespace grl;

REGISTER_CONFIGURABLE(LSPIPredictor)

void LSPIPredictor::request(ConfigurationRequest *config)
{
  Predictor::request(config);

  config->push_back(CRP("gamma", "Discount rate", gamma_));
  config->push_back(CRP("transitions", "Maximum number of transitions to store", max_samples_, CRP::Configuration, 1));
  config->push_back(CRP("iterations", "Number of policy improvement rounds per episode", iterations_, CRP::Configuration, 1));

  config->push_back(CRP("macro_batch_size", "Number of episodes/batches after which prediction is rebuilt. Use 0 for no rebuilds.", (int)macro_batch_size_));

  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.pair", "Projects observations-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Linear value function representation", representation_));
}

void LSPIPredictor::configure(Configuration &config)
{
  Predictor::configure(config);
  
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (LinearRepresentation*)config["representation"].ptr();
  
  gamma_ = config["gamma"];
  max_samples_ = config["transitions"];
  iterations_ = config["iterations"];
  macro_batch_size_ = config["macro_batch_size"];
  
  if (projector_->lifetime() != Projector::plIndefinite)
    throw Exception("predictor/lspi/projector must have indefinite lifetime");
  
  // Initialize memory
  reset();
}

void LSPIPredictor::reconfigure(const Configuration &config)
{
  Predictor::reconfigure(config);
  
  if (config.has("action") && config["action"].str() == "reset")
  {
    TRACE("Initializing transition store");
  
    transitions_.clear();
    macro_batch_counter_ = 0;
  }
}

LSPIPredictor &LSPIPredictor::copy(const Configurable &obj)
{
  const LSPIPredictor& fp = dynamic_cast<const LSPIPredictor&>(obj);
  
  transitions_ = fp.transitions_;
  macro_batch_counter_ = fp.macro_batch_counter_;

  return *this;
}

void LSPIPredictor::update(const Transition &transition)
{
  Predictor::update(transition);

  transitions_.push_back(CachedTransition(transition));
}

void LSPIPredictor::finalize()
{
  Predictor::finalize();

  // rebuilding predictor every macro_batch_size_ episodes or do not rebuild at all
  if (macro_batch_size_ && ((++macro_batch_counter_ % macro_batch_size_) == 0))
    rebuild();
}

void LSPIPredictor::rebuild()
{
  Matrix As = RowVector::Ones(representation_->size()).asDiagonal()*0.00001;
  ColumnVector b = ColumnVector::Zero(representation_->size());

  CRAWL("LSPI initialization");
 
  // Construct static part of A matrix, and b vector.
  for (size_t jj=0; jj < transitions_.size(); ++jj)
  {
    CachedTransition& t = transitions_[jj];
    
    if (!t.phi.size())
    {
      ProjectionPtr p = projector_->project(t.transition.prev_obs, t.transition.prev_action);
      
      IndexProjection *ip = dynamic_cast<IndexProjection*>(p.get());
      if (ip)
      {
        t.phi = ip->vector(representation_->size()).vector.matrix().transpose().sparseView(1., 0.00001);
      }
      else
      {
        VectorProjection *vp = dynamic_cast<VectorProjection*>(p.get());
        if (vp)
          t.phi = vp->vector.matrix().transpose().sparseView(1., 0.00001);
        else
          throw Exception("predictor/lspi/projector must return IndexProjecion or VectorProjection");
      }
    }
      
    As += t.phi * t.phi.transpose();
    b += t.phi * t.transition.reward;
  }
  
  for (size_t ii=0; ii < iterations_; ++ii)
  {
    CRAWL("LSPI iteration " << ii << ": constructing");
    
    Matrix A = As;
    
    for (size_t jj=0; jj < transitions_.size(); ++jj)
    {
      CachedTransition& t = transitions_[jj];

      if (t.transition.action.size())
      {
        if (!t.actions.size())
        {
          // Rebuild next state-action projections
          std::vector<Vector> variants;
          discretizer_->options(t.transition.obs, &variants);
          projector_->project(t.transition.obs, variants, &t.actions);
          t.nextphi = Eigen::SparseMatrix<double>(representation_->size(), t.actions.size());
          
          for (size_t kk=0; kk < t.actions.size(); ++kk)
          {
            IndexProjection *ip = dynamic_cast<IndexProjection*>(t.actions[kk].get());
            if (ip)
            {
              t.nextphi.col(kk) = ip->vector(representation_->size()).vector.matrix().transpose().sparseView(1., 0.00001);
            }
            else
            {
              VectorProjection *vp = dynamic_cast<VectorProjection*>(t.actions[kk].get());
              if (vp)
                t.nextphi.col(kk) = vp->vector.matrix().transpose().sparseView(1., 0.00001);
              else
                throw bad_param("predictor/lspi/projector must return IndexProjecion or VectorProjection");
            }
          }
        }
        
        // Find value of best action
        Vector value;
        double bestv=-std::numeric_limits<double>::infinity();
        size_t besta=0;
        for (size_t kk=0; kk < t.actions.size(); ++kk)
        {
          double v = representation_->read(t.actions[kk], &value, NULL);
          if (v > bestv)
          {
            bestv = v;
            besta = kk;
          }
        }
        
        // Add to A matrix
        A -= gamma_ * t.phi * t.nextphi.col(besta).transpose();
      }
    }
    
    // Solve
    CRAWL("LSPI iteration " << ii << ": solving");
    representation_->params() = A.partialPivLu().solve(b).array();
  }
}
