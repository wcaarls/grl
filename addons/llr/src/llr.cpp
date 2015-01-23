/** \file llr.cpp
 * \brief Locally weighted regression representation source file.
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

#include <grl/representations/llr.h>
#include <grl/projections/sample.h>

using namespace grl;

REGISTER_CONFIGURABLE(LLRRepresentation)

void LLRRepresentation::request(ConfigurationRequest *config)
{
  config->push_back(CRP("outputs", "Number of output dimensions", outputs_, CRP::System, 1));
  config->push_back(CRP("ridge", "Ridge regression (Tikhonov) factor", ridge_regression_factor_));

  config->push_back(CRP("projector", "projector/sample", "Projector used to generate input for this representation", projector_));
}

void LLRRepresentation::configure(Configuration &config)
{
  projector_ = (SampleProjector*)config["projector"].ptr();
  
  ridge_regression_factor_ = config["ridge"];
  outputs_ = config["outputs"];
}

void LLRRepresentation::reconfigure(const Configuration &config)
{
}

LLRRepresentation *LLRRepresentation::clone() const
{
  return NULL;
}

double LLRRepresentation::read(const ProjectionPtr &projection, Vector *result) const
{
  SampleProjection *p = dynamic_cast<SampleProjection*>(projection.get());
  grl_assert(p);
  
  result->clear();

  // Convert query
  RowVector q(p->query.size()+1);
  
  for (size_t ii=0; ii < p->query.size(); ++ii)
    q[ii] = p->query[ii];
  q[p->query.size()] = 1.;

  // Fill matrices A and b
  Matrix A(p->indices.size(), p->query.size()+1),
         b(p->indices.size(), outputs_);
  
  {
    Guard guard(*p->store);
    for (size_t ii=0; ii < p->indices.size(); ++ii)
    {
      for (size_t jj=0; jj < p->query.size(); ++jj)
        A(ii, jj) = (*p->store)[p->indices[ii]]->in[jj]*p->weights[ii];
      A(ii, p->query.size()) = p->weights[ii];
      
      for (size_t jj=0; jj < outputs_; ++jj)
        b(ii, jj) = (*p->store)[p->indices[ii]]->out[jj]*p->weights[ii];
    }
  }
  
  // Solve with ATA*x = ATb with QR
  Matrix At = A.transpose();  
  Matrix ATAL = At*A + Matrix::Identity(A.cols(), A.cols())*ridge_regression_factor_;
  Eigen::LLT<Matrix> decompATAL(ATAL);
  if (decompATAL.info() != Eigen::Success)
    return 0.;

  Matrix r = decompATAL.solve(At);
  if (decompATAL.info() != Eigen::Success)
    return 0.;

  RowVector y = q*(r*b);
  
  result->resize(outputs_);
  for (size_t ii=0; ii < outputs_; ++ii)
    (*result)[ii] = y[ii];
 
  return y[0];
}

void LLRRepresentation::write(const ProjectionPtr projection, const Vector &target, double alpha)
{
  SampleProjection *p = dynamic_cast<SampleProjection*>(projection.get());
  grl_assert(p);
  
  // Push query on store
  Sample *sample = new Sample();

  for (size_t ii=0; ii < p->query.size(); ++ii)
    sample->in[ii] = p->query[ii];
    
  for (size_t ii=0; ii < target.size(); ++ii)
    sample->out[ii] = target[ii];
    
  sample->relevance = 1.;
  
  projector_->push(sample);
}

void LLRRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
}
                