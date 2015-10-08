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

#include <map>

#include <grl/representations/llr.h>
#include <grl/projections/sample.h>

using namespace grl;

REGISTER_CONFIGURABLE(LLRRepresentation)

void LLRRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("ridge", "Ridge regression (Tikhonov) factor", ridge_regression_factor_));
  config->push_back(CRP("order", "Order of regression model", order_, CRP::Configuration, 0, 1));
  config->push_back(CRP("input_nominals", "Vector indicating which input dimensions are nominal", input_nominals_));
  config->push_back(CRP("output_nominals", "Vector indicating which output dimensions are nominal", output_nominals_));

  if (role == "action")
  {
    config->push_back(CRP("outputs", "int.action_dims", "Number of output dimensions", outputs_, CRP::System, 1));
    config->push_back(CRP("output_min", "vector.action_min", "Lower output limit", min_, CRP::System));
    config->push_back(CRP("output_max", "vector.action_max", "Upper output limit", max_, CRP::System));
    config->push_back(CRP("projector", "projector/sample.observation", "Projector used to generate input for this representation", projector_));
  }
  else if (role == "transition")
  {
    config->push_back(CRP("outputs", "int.observation_dims+2", "Number of output dimensions", outputs_, CRP::System, 1));
    config->push_back(CRP("output_min", "Lower output limit", min_, CRP::System));
    config->push_back(CRP("output_max", "Upper output limit", max_, CRP::System));
    config->push_back(CRP("projector", "projector/sample.pair", "Projector used to generate input for this representation", projector_));
  }
  else
  {
    config->push_back(CRP("outputs", "Number of output dimensions", outputs_, CRP::System, 1));
    config->push_back(CRP("output_min", "Lower output limit", min_, CRP::System));
    config->push_back(CRP("output_max", "Upper output limit", max_, CRP::System));
    
    if (role == "value/state")
      config->push_back(CRP("projector", "projector/sample.observation", "Projector used to generate input for this representation", projector_));
    else if (role == "value/action")
      config->push_back(CRP("projector", "projector/sample.pair", "Projector used to generate input for this representation", projector_));
    else    
      config->push_back(CRP("projector", "projector/sample", "Projector used to generate input for this representation", projector_));
  }
}

void LLRRepresentation::configure(Configuration &config)
{
  projector_ = (SampleProjector*)config["projector"].ptr();
  
  ridge_regression_factor_ = config["ridge"];
  outputs_ = config["outputs"];
  order_ = config["order"];
  input_nominals_ = config["input_nominals"];
  output_nominals_ = config["output_nominals"];

  if (output_nominals_.size() && output_nominals_.size() != outputs_)
    throw bad_param("representation/lrr:output_nominals");

  min_ = config["output_min"];
  max_ = config["output_max"];

  if (!min_.size())
    min_ = ConstantVector(outputs_, -DBL_MAX);
  if (min_.size() != outputs_)
    throw bad_param("representation/llr:output_min");
  if (!max_.size())
    max_ = ConstantVector(outputs_, DBL_MAX);
  if (max_.size() != outputs_)
    throw bad_param("representation/llr:output_max");
}

void LLRRepresentation::reconfigure(const Configuration &config)
{
}

LLRRepresentation *LLRRepresentation::clone() const
{
  return NULL;
}

double LLRRepresentation::read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
{
  SampleProjection *p = dynamic_cast<SampleProjection*>(projection.get());
  
  if (!p)
    throw Exception("representation/llr requires projector/sample");
  
  *result = Vector();

  // Get input and output matrices
  Matrix A, b;
  getMatrices(p, &A, &b);
  
  if (!A.rows())
    return 0.;

  if (!order_)
  {
    // Zeroth-order regression (averaging)
    double weight = 0;
    *result = ConstantVector(outputs_, 0.);
    
    for (size_t ii=0; ii < A.rows(); ++ii)
    {
      double w = A(ii, A.cols()-1);
      for (size_t jj=0; jj < outputs_; ++jj)
        (*result)[jj] += b(ii, jj)*w;
      weight += w;
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

  Matrix r = estimateModel(A);
  Matrix x = r*b;
  RowVector y = q*x;

  // Calculate variance
  if (stddev)
  {
    // Get weights column
    ColumnVector wsqr = A.col(p->query.size()).array().square();

    // Calculate number of data points
    double n_LWR = wsqr.sum();
    
    // Calculate number of free parameters
    double p_LWR = (wsqr.array()*(A*r).diagonal().array()).sum();

    // Calculate error matrix (samples x outputs)
    Matrix e = A*x-b;
    
    // Calculate variance
    RowVector sigma = (e.array().square().colwise().sum()/(n_LWR-p_LWR)).sqrt();

    // Convert output
    stddev->resize(outputs_);
    for (size_t ii=0; ii < outputs_; ++ii)
      (*stddev)[ii] = sigma[ii];
  }

  // Convert output
  result->resize(outputs_);
  for (size_t ii=0; ii < outputs_; ++ii)
    (*result)[ii] = y[ii];
    
  // Avoid extrapolation
  RowVector mib(outputs_), mab(outputs_);
  for (size_t ii=0; ii < outputs_; ++ii)
  {
    mib[ii] = std::numeric_limits<double>::infinity();
    mab[ii] = -std::numeric_limits<double>::infinity();
  }
  
  {
    Guard guard(*p->store);
    for (size_t ii=0; ii < b.rows(); ++ii)
    {
      for (size_t jj=0; jj < outputs_; ++jj)
      {
        double o = b(ii, jj);
        mib[jj] = fmin(mib[jj], o);
        mab[jj] = fmax(mab[jj], o);
      }
    }
  }

  for (size_t ii=0; ii < outputs_; ++ii)
  {
    (*result)[ii] = fmin(fmax((*result)[ii], mib[ii]), mab[ii]);
    (*result)[ii] = fmin(fmax((*result)[ii], min_[ii]), max_[ii]);
  }
  
  return (*result)[0];
}

void LLRRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  SampleProjection *p = dynamic_cast<SampleProjection*>(projection.get());
  if (!p)
    throw Exception("representation/llr requires projector/sample");
    
  if (target.size() != outputs_)
    throw bad_param("representation/llr:outputs");
    
  if (alpha.size() != target.size())
    throw Exception("Learning rate vector does not match target vector");

  // Prepare sample
  Sample *sample;
  if (!p->sample)
  {
    sample = new Sample();

    for (size_t ii=0; ii < p->query.size(); ++ii)
      sample->in[ii] = p->query[ii];
  }
  else
    sample = p->sample;
  
  if (prod(alpha) != 1)
  {
    // Reinforcement learning: move sample neighborhood towards target value
    Vector out;
    read(projection, &out, NULL);
    
    if (!out.size())
      out = ConstantVector(target.size(), 0.);
    
    Vector delta = target-out;
    
    // Update new sample
    for (size_t ii=0; ii < target.size(); ++ii)
      sample->out[ii] = fmin(fmax(out[ii] + alpha[ii]*delta[ii], min_[ii]), max_[ii]);

    // Update neighbors      
    update(projection, alpha*delta);
  }
  else
  {
    // Supervised learning: just add sample with target value
    for (size_t ii=0; ii < target.size(); ++ii)
      sample->out[ii] = fmin(fmax(target[ii], min_[ii]), max_[ii]);
  }
  
  if (!p->sample)
  {
    // Determine sample relevance
    if (p->neighbors.size())
    {
      Guard guard(*p->store);
      Sample *neighbor = p->neighbors[0];

      // Relevance based on euclidean distance
      sample->relevance = 0;
      for (size_t ii=0; ii < p->query.size(); ++ii)
        sample->relevance += pow(sample->in[ii]-neighbor->in[ii], 2);
    }
    else
      sample->relevance = 1.;
      
    // Don't add identical samples
    if (sample->relevance > 0.000001)
    {
      projector_->push(sample);
      p->sample = sample;
    }
    else
      delete sample;
  }
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
    for (size_t ii=0; ii < p->neighbors.size(); ++ii)
      for (size_t jj=0; jj < outputs_; ++jj)
        p->neighbors[ii]->out[jj] = fmin(fmax(p->neighbors[ii]->out[jj] + delta[jj]*p->weights[ii], min_[jj]), max_[jj]);
  }
}

void LLRRepresentation::finalize()
{
  projector_->finalize();
}

Matrix LLRRepresentation::jacobian(const ProjectionPtr projection) const
{
  SampleProjection *p = dynamic_cast<SampleProjection*>(projection.get());
  
  if (!p)
    throw Exception("representation/llr requires projector/sample");

  if (!order_)
    return grl::Matrix::Zero(outputs_, p->query.size());
  
  // Get input and output matrices
  Matrix A, b;
  getMatrices(p, &A, &b);
  
  if (!A.rows())
    return grl::Matrix();
  
  // Because the model is linear, the Jacobian is simply the model matrix.
  Matrix x = (estimateModel(A)*b).transpose();
  grl::Matrix J(outputs_, p->query.size());

  // Note that x also includes the constant term, which is discarded here.
  for (size_t rr=0; rr < outputs_; ++rr)
    for (size_t cc=0; cc < p->query.size(); ++cc)
      J(rr, cc) = x(rr, cc);
      
  return J;  
}

void LLRRepresentation::getMatrices(const SampleProjection *p, Matrix *A, Matrix *b) const
{
  std::vector<bool> eligible(p->neighbors.size(), true);

  // Filter samples with different input nominal than query
  if (input_nominals_.size())
  {
    Guard guard(*p->store);

    if (input_nominals_.size() != p->query.size())
      throw bad_param("representation/lrr:input_nominals");
    
    double query_nominal=0;
    for (size_t jj=0; jj < p->query.size(); ++jj)
      query_nominal +=p->query[jj]*input_nominals_[jj];
    
    for (size_t ii=0; ii < p->neighbors.size(); ++ii)
    {
      double *in = p->neighbors[ii]->in;
      double nominal = 0;
      
      for (size_t jj=0; jj < p->query.size(); ++jj)
        nominal += in[jj]*input_nominals_[jj];
    
      if (nominal != query_nominal)
        eligible[ii] = false;
    }
  }

  // Filter samples with different than most likely output nominal
  if (output_nominals_.size())  
  {
    Guard guard(*p->store);

    // Determine output nominals
    Vector nominals(p->neighbors.size());
    
    for (size_t ii=0; ii < p->neighbors.size(); ++ii)
    {
      double *out = p->neighbors[ii]->out;
      double nominal = 0;
    
      for (size_t jj=0; jj < outputs_; ++jj)
        nominal += out[jj]*output_nominals_[jj];
      nominals[ii] = nominal;
    }
  
    // Determine most likely output nominal
    std::map<double, double> nominal_weights;
  
    for (size_t ii=0; ii < p->neighbors.size(); ++ii)
      nominal_weights[nominals[ii]] += p->weights[ii];
    
    double out_nominal = nominal_weights.begin()->first;
    for (std::map<double, double>::iterator ii=nominal_weights.begin(); ii != nominal_weights.end(); ++ii)
      if (ii->second > nominal_weights[out_nominal])
        out_nominal = ii->first;

    for (size_t ii=0; ii < nominals.size(); ++ii)
      if (nominals[ii] != out_nominal)
        eligible[ii] = false;
  }
  
  size_t eligibles=0;
  for (size_t ii=0; ii < p->neighbors.size(); ++ii)
    eligibles += eligible[ii];

  // Fill matrices A and b
  *A = Matrix(eligibles, p->query.size()+1),
  *b = Matrix(eligibles, outputs_);
         
  {
    Guard guard(*p->store);
    for (size_t ii=0, i2=0; ii < p->neighbors.size(); ++ii)
    {
      if (!eligible[ii])
        continue;
    
      if (order_)
        for (size_t jj=0; jj < p->query.size(); ++jj)
          (*A)(i2, jj) = p->neighbors[ii]->in[jj]*p->weights[ii];
      (*A)(i2, p->query.size()) = p->weights[ii];
      
      for (size_t jj=0; jj < outputs_; ++jj)
      {
        double o = p->neighbors[ii]->out[jj];
        (*b)(i2, jj) = o*p->weights[ii];
      }
      
      // Increment when we have processed an eligible neighbor
      i2++;
    }
  }
}

LLRRepresentation::Matrix LLRRepresentation::estimateModel(const Matrix &A) const
{
  // Solve with ATA*x = ATb with Cholesky decomposition
  Matrix At = A.transpose();  
  Matrix ATAL = At*A + Matrix::Identity(A.cols(), A.cols())*ridge_regression_factor_;
  Eigen::LLT<Matrix> decompATAL(ATAL);
  if (decompATAL.info() != Eigen::Success)
    return Matrix();

  Matrix r = decompATAL.solve(At);
  if (decompATAL.info() != Eigen::Success)
    return Matrix();
    
  return r;
}
