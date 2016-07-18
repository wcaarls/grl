/** \file ann.cpp
 * \brief Artificial neural network representation source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-06-26
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

#include <string.h>
#include <grl/representations/ann.h>

using namespace grl;

REGISTER_CONFIGURABLE(ANNRepresentation)

void ANNRepresentation::request(const std::string &role, ConfigurationRequest *config)
{
  if (role == "action")
  {
    config->push_back(CRP("inputs", "int.observation_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    config->push_back(CRP("outputs", "int.action_dims", "Number of output dimensions", (int)outputs_, CRP::System, 1));
  }
  else
  {
    if (role == "transition" || role == "value/action")
      config->push_back(CRP("inputs", "int.observation_dims+int.action_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    else if (role == "value/state")
      config->push_back(CRP("inputs", "int.observation_dims", "Number of input dimensions", (int)inputs_, CRP::System, 1));
    else
      config->push_back(CRP("inputs", "Number of input dimensions", (int)inputs_, CRP::System, 1));
      
    if (role == "transition")
      config->push_back(CRP("outputs", "int.observation_dims+2", "Number of output dimensions", (int)outputs_, CRP::System, 1));
    else
      config->push_back(CRP("outputs", "Number of output dimensions", (int)outputs_, CRP::System, 1));
  }
  
  config->push_back(CRP("hiddens", "Number of hidden nodes per layer", hiddens_, CRP::Configuration));

  config->push_back(CRP("eta", "Learning rate (0=RPROP)", eta_, CRP::Configuration, 0., 2.));
}

void ANNRepresentation::configure(Configuration &config)
{
  inputs_ = config["inputs"];
  outputs_ = config["outputs"];
  hiddens_ = config["hiddens"].v();

  layers_.resize(hiddens_.size()+2);
  layers_[0].size = inputs_;
  layers_[layers_.size()-1].size = outputs_;
  
  for (size_t ii=0; ii < hiddens_.size(); ++ii)
  {
    if (round(hiddens_[ii]) <= 0)
      throw bad_param("representation/parameterized/ann:hiddens");
      
    layers_[ii+1].size = round(hiddens_[ii]);
  }
    
  eta_ = config["eta"];
    
  // Initialize memory
  reset();
}

void ANNRepresentation::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    size_t sz = 0;
    for (size_t ii=1; ii < layers_.size(); ++ii)
      sz += (layers_[ii-1].size+1)*layers_[ii].size;
    params_ = Vector::Random(sz)*0.01;
    
    sz = 0;
    for (size_t ii=1; ii < layers_.size(); ++ii)
    {
      new (&layers_[ii].W) Eigen::Map<Eigen::MatrixXd>(&params_.data()[sz], layers_[ii-1].size+1, layers_[ii].size);
      
      layers_[ii].delta = layers_[ii].activation = Matrix();
      layers_[ii].Delta = Matrix::Zero(layers_[ii-1].size+1, layers_[ii].size);

      layers_[ii].eta = Matrix::Ones(layers_[ii-1].size+1, layers_[ii].size)*0.1;
      layers_[ii].prev_Delta = Matrix::Zero(layers_[ii-1].size+1, layers_[ii].size);

      sz += (layers_[ii-1].size+1)*layers_[ii].size;
    }
    
    error_ = 0;
    samples_ = 0;
  }
}

ANNRepresentation *ANNRepresentation::clone() const
{
  ANNRepresentation *ann = new ANNRepresentation(*this);
  
  // Remap parameter vector to weight matrices
  size_t sz = 0;
  for (size_t ii=1; ii < ann->layers_.size(); ++ii)
  {
    new (&ann->layers_[ii].W) Eigen::Map<Eigen::MatrixXd>(&ann->params_.data()[sz], ann->layers_[ii-1].size+1, ann->layers_[ii].size);
    sz += (ann->layers_[ii-1].size+1)*ann->layers_[ii].size;
  }

  return ann;
}

double ANNRepresentation::read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    Eigen::VectorXd act = vp->vector;
    for (int ii=1; ii < layers_.size(); ++ii)
    {
      Eigen::VectorXd net = layers_[ii].W.transpose().leftCols(layers_[ii-1].size)*act +
                            layers_[ii].W.transpose().rightCols<1>(); // Bias

      if (ii == layers_.size()-1)
        act = net; // Linear activation on output neurons
      else
        act = activate(net);
    }
    
    *result = act;
  }
  else
    throw Exception("representation/parameterized/ann requires a projector returning a VectorProjection");
    
  if (stddev) *stddev = Vector();
  
  return (*result)[0];
}

void ANNRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    // TODO: allow to call backprop with alpha
    if (prod(alpha) == 1)
    {
      backprop(vp->vector.transpose(), target);
    }
    else
    {
      Vector v;
      read(projection, &v, NULL);
      backprop(vp->vector.transpose(), (alpha*target + (1-alpha)*v).transpose());
    }
  }
  else
    throw Exception("representation/parameterized/ann requires a projector returning a VectorProjection");
}

void ANNRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    // TODO: allow to call backprop with error
    Vector v;
    read(projection, &v, NULL);
    backprop(vp->vector.transpose(), (v+delta).transpose());
  }
  else
    throw Exception("representation/parameterized/ann requires a projector returning a VectorProjection");
}

void ANNRepresentation::finalize()
{
  for (int ii=1; ii < layers_.size(); ++ii)
  {
    if (eta_)
    {
      // Gradient descent
      layers_[ii].W -= eta_*layers_[ii].Delta/samples_;
    }
    else
    {
      // RPROP
      layers_[ii].eta = ((layers_[ii].Delta.array()*layers_[ii].prev_Delta.array())>0).select(layers_[ii].eta*1.2, layers_[ii].eta*0.5);
      layers_[ii].W -= (layers_[ii].Delta.array()>0).select(layers_[ii].eta, -layers_[ii].eta);
      layers_[ii].prev_Delta = layers_[ii].Delta;
    }
    
    layers_[ii].Delta = Matrix::Zero(layers_[ii-1].size+1, layers_[ii].size);
  }
  
  CRAWL("Error: " << error_/samples_);
  
  error_ = 0;
  samples_ = 0;
}

void ANNRepresentation::backprop(const Matrix &in, const Matrix &out)
{
  // Feed forward, to get activations
  layers_[0].activation = in;
  for (size_t ii=1; ii < layers_.size(); ++ii)
  {
    Matrix net = layers_[ii].W.transpose().leftCols(layers_[ii-1].size)*layers_[ii-1].activation +
                 layers_[ii].W.transpose().rightCols<1>(); // Bias
                 
    if (ii == layers_.size()-1)
      layers_[ii].activation = net; // Linear activation on output neurons
    else
      layers_[ii].activation = activate(net);
  }
  
  // Update error
  error_ += (layers_[layers_.size()-1].activation - out).array().square().sum();

  // Backpropagation
  for (int ii=layers_.size()-1; ii > 0; --ii)
  {
    if (ii == layers_.size()-1)
      layers_[ii].delta = layers_[ii].activation - out; // Linear activation on output neurons
    else
      layers_[ii].delta = (layers_[ii+1].W*layers_[ii+1].delta).topRows(layers_[ii+1].size).cwiseProduct(dactivate(layers_[ii].activation));
      
    layers_[ii].Delta.topRows(layers_[ii-1].size) += layers_[ii-1].activation*layers_[ii].delta.transpose();
    layers_[ii].Delta.bottomRows<1>() += layers_[ii].delta.transpose(); // Bias
  }
  
  samples_ += in.cols();
}
