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
  config->push_back(CRP("order", "Order of regression model", order_, CRP::Configuration, 0, 1));

  config->push_back(CRP("projector", "projector/sample", "Projector used to generate input for this representation", projector_));
}

void LLRRepresentation::configure(Configuration &config)
{
  projector_ = (SampleProjector*)config["projector"].ptr();
  
  ridge_regression_factor_ = config["ridge"];
  outputs_ = config["outputs"];
  order_ = config["order"];
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
  
  if (!p)
    throw Exception("representation/llr requires projector/sample");
  
  result->clear();

  if (p->indices.empty())
    return 0.;
  
  if (!order_)
  {
    // Zeroth-order regression (averaging)
    Guard guard(*p->store);
    
    double weight = 0;
    result->resize(outputs_, 0.);
    
    for (size_t ii=0; ii < p->indices.size(); ++ii)
    {
      weight += p->weights[ii];
      for (size_t jj=0; jj < outputs_; ++jj)
        (*result)[jj] += (*p->store)[p->indices[ii]]->out[jj]*p->weights[ii];
    }
    
    if (weight)
      for (size_t jj=0; jj < outputs_; ++jj)
        (*result)[jj] /= weight;
      
    return (*result)[0];
  }
  
  // Convert query
  RowVector q(p->query.size()+1);
  
  for (size_t ii=0; ii < p->query.size(); ++ii)
    q[ii] = p->query[ii];
  q[p->query.size()] = 1.;
  
  // Fill matrices A and b
  Matrix A(p->indices.size(), p->query.size()+1),
         b(p->indices.size(), outputs_);
         
  RowVector mib(outputs_), mab(outputs_);
  for (size_t ii=0; ii < outputs_; ++ii)
  {
    mib[ii] = std::numeric_limits<double>::infinity();
    mab[ii] = -std::numeric_limits<double>::infinity();
  }
  
  {
    Guard guard(*p->store);
    for (size_t ii=0; ii < p->indices.size(); ++ii)
    {
      for (size_t jj=0; jj < p->query.size(); ++jj)
        A(ii, jj) = (*p->store)[p->indices[ii]]->in[jj]*p->weights[ii];
      A(ii, p->query.size()) = p->weights[ii];
      
      for (size_t jj=0; jj < outputs_; ++jj)
      {
        double o = (*p->store)[p->indices[ii]]->out[jj];
        b(ii, jj) = o*p->weights[ii];
        mib[jj] = fmin(mib[jj], o);
        mab[jj] = fmax(mab[jj], o);
      }
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
  
  // Convert output
  result->resize(outputs_);
  for (size_t ii=0; ii < outputs_; ++ii)
    (*result)[ii] = y[ii];
    
  // Avoid extrapolation
  if (p->indices.size())
    for (size_t ii=0; ii < outputs_; ++ii)
      (*result)[ii] = fmin(fmax((*result)[ii], mib[ii]), mab[ii]);
  
  return (*result)[0];
}

void LLRRepresentation::write(const ProjectionPtr projection, const Vector &target, double alpha)
{
  SampleProjection *p = dynamic_cast<SampleProjection*>(projection.get());
  if (!p)
    throw Exception("representation/llr requires projector/sample");
    
  if (target.size() != outputs_)
    throw bad_param("representation/llr:outputs");
  
  // Push query on store
  Sample *sample = new Sample();

  for (size_t ii=0; ii < p->query.size(); ++ii)
    sample->in[ii] = p->query[ii];
  
  if (alpha < 1)
  {
    // Reinforcement learning: move sample neighborhood towards target value
    Vector out;
    read(projection, &out);
    
    if (out.empty())
      out.resize(target.size(), 0.);
    
    Vector delta = target-out;
    
    // Update new sample
    for (size_t ii=0; ii < target.size(); ++ii)
      sample->out[ii] = out[ii] + alpha*delta[ii];

    // Update neighbors      
    update(projection, alpha*delta);
  
    // Determine sample relevance
    if (p->indices.size())
    {
      Guard guard(*p->store);
      Sample *neighbor = (*p->store)[p->indices[0]];
        
      // Relevance based on euclidean distance
      sample->relevance = 0;
      for (size_t ii=0; ii < p->query.size(); ++ii)
        sample->relevance += pow(sample->in[ii]-neighbor->in[ii], 2);
    }
    else
      sample->relevance = 1.;
  }
  else
  {
    // Supervised learning: just add sample with target value
    for (size_t ii=0; ii < target.size(); ++ii)
      sample->out[ii] = target[ii];
    sample->relevance = 1.;
  }
  
  // Don't add identical samples
  if (sample->relevance > 0.001)
    projector_->push(sample);
  else
    delete sample;
}

void LLRRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  SampleProjection *p = dynamic_cast<SampleProjection*>(projection.get());
  if (!p)
    throw Exception("representation/llr requires projector/sample");
    
  if (delta.size() != outputs_)
    throw bad_param("representation/llr:outputs");

  {
    Guard guard(*p->store);
  
    // Less contributing neighbors are updated less
    for (size_t ii=0; ii < p->indices.size(); ++ii)
      for (size_t jj=0; jj < outputs_; ++jj)
        (*p->store)[p->indices[ii]]->out[jj] += delta[jj]*p->weights[ii];
  }
}
                