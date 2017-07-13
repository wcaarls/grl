/** \file tensorflow.cc
 * \brief Tensorflow representation source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-31
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

#include <vector>

#include "representation.h"

using tensorflow::Tensor;
using tensorflow::TensorShape;
using tensorflow::Status;

using namespace grl;

REGISTER_CONFIGURABLE(TensorFlowRepresentation)

void TensorFlowRepresentation::request(const std::string &role, ConfigurationRequest *config)
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
  
  config->push_back(CRP("file", "TensorFlow graph stored in binary protocol buffer", file_));
  
  config->push_back(CRP("input_layer", "Name of input layer placeholder", input_layer_));
  config->push_back(CRP("output_layer", "Name of output layer tensor", output_layer_));
  config->push_back(CRP("target", "Name of output layer target placeholder", output_target_));
  config->push_back(CRP("sample_weights", "Name of sample weights placeholder", sample_weights_));
  config->push_back(CRP("learning_phase", "Name of learning phase placeholder", learning_phase_));

  config->push_back(CRP("init_node", "Name of node to run to initialize weights", init_node_));
  config->push_back(CRP("update_node", "Name of node to run to update weights", update_node_));
}

void TensorFlowRepresentation::configure(Configuration &config)
{
  inputs_ = config["inputs"];
  outputs_ = config["outputs"];
  file_ = config["file"].str();
  if (file_.empty())
    throw bad_param("representation/tensorflow:file");
  
  NOTICE("Loading TensorFlow model " << file_);

  Status load_graph_status = ReadBinaryProto(tensorflow::Env::Default(), file_, &graph_def_);
  if (!load_graph_status.ok()) {
    ERROR(load_graph_status.ToString());
    throw bad_param("representation/tensorflow:file");
  }
  
  session_ = tensorflow::NewSession(tensorflow::SessionOptions());
  Status session_create_status = session_->Create(graph_def_);
  if (!session_create_status.ok()) {
    ERROR(load_graph_status.ToString());
    throw bad_param("representation/tensorflow:file");
  }

  for (int i = 0; i < graph_def_.node_size(); i++)
  {
    auto n = graph_def_.node(i);
    
    if (input_layer_.empty() && n.name().find("input") != std::string::npos)
      input_layer_ = n.name();

    if (output_target_.empty() && n.name().find("target") != std::string::npos)
      output_target_ = n.name();
      
    if (sample_weights_.empty() && n.name().find("sample_weights") != std::string::npos)
      sample_weights_ = n.name();

    if (learning_phase_.empty() && n.name().find("learning_phase") != std::string::npos)
      learning_phase_ = n.name();

    if (init_node_.empty() && n.name() == "init")
      init_node_ = n.name();

    if (update_node_.empty() && n.name() == "group_deps")
      update_node_ = n.name();

    if (output_layer_.empty() && n.name() == "group_deps_1")
      output_layer_ = n.input(0).substr(1);
      
    if (n.op() == "Assign" && n.input_size() == 2 && n.input(1).find("Placeholder") != std::string::npos)
    {
      weights_node_.push_back(n.name());
      weights_read_.push_back(n.input(0) + "/read");
      weights_write_.push_back(n.input(1));
    }
  }
  
  if (input_layer_.empty())
    throw bad_param("representation/tensorflow:input_layer");

  if (output_layer_.empty())
    throw bad_param("representation/tensorflow:output_layer");

  if (output_target_.empty())
    throw bad_param("representation/tensorflow:target");

  if (update_node_.empty())
    throw bad_param("representation/tensorflow:update_node");
    
  INFO("Model loaded");
  INFO("  Input layer placeholder   : " << input_layer_);
  INFO("  Output layer tensor       : " << output_layer_);
  INFO("  Target placeholder        : " << output_target_);
  INFO("  Sample weights placeholder: " << sample_weights_);
  INFO("  Learning phase placeholder: " << learning_phase_);
  INFO("  Weight tensors            : " << weights_read_);
  INFO("  Weight placeholders       : " << weights_write_);
  INFO("  Weight nodes              : " << weights_node_);
  INFO("  Init node                 : " << init_node_);
  INFO("  Update node               : " << update_node_);
  
  // Initialize weights
  reset();
}

void TensorFlowRepresentation::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    if (!init_node_.empty())
    {
      INFO("Initializing weights");
      
      Status run_status = session_->Run({}, {}, {init_node_}, NULL);
      if (!run_status.ok()) {
        ERROR(run_status.ToString());
        throw bad_param("representation/tensorflow:file");
      }
    }
    
    batch_.clear();
    
    // Get parameter vector size
    params();
  }
}

// Not reentrant
TensorFlowRepresentation &TensorFlowRepresentation::copy(const Configurable &obj)
{
  const TensorFlowRepresentation &tfr = dynamic_cast<const TensorFlowRepresentation&>(obj);
  
  setParams(tfr.params());

  return *this;
}

double TensorFlowRepresentation::read(const ProjectionPtr &projection, Vector *result, Vector *stddev) const
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    Tensor input(tensorflow::DT_FLOAT, TensorShape({1, (int)vp->vector.size()}));
    auto input_map = input.tensor<float, 2>();
    for (size_t ii=0; ii < vp->vector.size(); ++ii)
      input_map(0, ii) = vp->vector[ii];

    Tensor learning(tensorflow::DT_BOOL, TensorShape());
    learning.scalar<bool>()() = false;

    std::vector<std::pair<std::string, tensorflow::Tensor>> inputs = {{input_layer_, input}};
    if (!learning_phase_.empty())
      inputs.push_back({learning_phase_, learning});
    
    std::vector<Tensor> outputs;
    
    Status run_status = session_->Run(inputs, {output_layer_}, {}, &outputs);
    if (!run_status.ok()) {
      ERROR(run_status.ToString());
      throw Exception("Could not run prediction graph");
    }
    
    auto output_map = outputs[0].tensor<float, 2>();
    result->resize(output_map.dimension(1));
    for (size_t ii=0; ii < result->size(); ++ii)
      (*result)[ii] = output_map(0, ii);
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
    
  if (stddev) *stddev = Vector();
  
  return (*result)[0];
}

void TensorFlowRepresentation::write(const ProjectionPtr projection, const Vector &target, const Vector &alpha)
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    if (prod(alpha) == 1)
    {
      batch_.push_back({vp->vector, target});
    }
    else
    {
      Vector v;
      read(projection, &v, NULL);
      batch_.push_back({vp->vector, (1-alpha)*v + alpha*target});
    }
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
}

void TensorFlowRepresentation::update(const ProjectionPtr projection, const Vector &delta)
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    Vector v;
    read(projection, &v, NULL);
    batch_.push_back({vp->vector, v+delta});
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
}

void TensorFlowRepresentation::finalize()
{
  Tensor input(tensorflow::DT_FLOAT, TensorShape({(int)batch_.size(), (int)batch_[0].first.size()}));
  auto input_map = input.tensor<float, 2>();
  for (size_t ii=0; ii < batch_.size(); ++ii)
    for (size_t jj=0; jj < batch_[ii].first.size(); ++jj)
      input_map(ii, jj) = batch_[ii].first[jj];

  Tensor learning(tensorflow::DT_BOOL, TensorShape());
  learning.scalar<bool>()() = true;

  Tensor target(tensorflow::DT_FLOAT, TensorShape({(int)batch_.size(), (int)batch_[0].second.size()}));
  auto target_map = target.tensor<float, 2>();
  for (size_t ii=0; ii < batch_.size(); ++ii)
    for (size_t jj=0; jj < batch_[ii].second.size(); ++jj)
      target_map(ii, jj) = batch_[ii].second[jj];

  Tensor weights(tensorflow::DT_FLOAT, TensorShape({(int)batch_.size()}));
  auto weight_map = weights.tensor<float, 1>();
  for (size_t ii=0; ii < batch_.size(); ++ii)
    weight_map(ii) = 1.;
    
  std::vector<std::pair<std::string, tensorflow::Tensor>> inputs = {{input_layer_, input}, {output_target_, target}};
  if (!learning_phase_.empty())
    inputs.push_back({learning_phase_, learning});
  if (!sample_weights_.empty())
    inputs.push_back({sample_weights_, weights});

  // Training
  Status run_status = session_->Run(inputs, {}, {update_node_}, NULL);
  if (!run_status.ok()) {
    ERROR(run_status.ToString());
    throw Exception("Could not run learning graph");
  }
  
  batch_.clear();
}

void TensorFlowRepresentation::batch(size_t sz)
{
  input_ = Tensor(tensorflow::DT_FLOAT, TensorShape({(int)sz, (int)inputs_}));
  output_ = Tensor(tensorflow::DT_FLOAT, TensorShape({(int)sz, (int)outputs_}));
  target_ = Tensor(tensorflow::DT_FLOAT, TensorShape({(int)sz, (int)outputs_}));
  
  counter_ = 0;
}

void TensorFlowRepresentation::enqueue(const ProjectionPtr &projection)
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    auto input_map = input_.tensor<float, 2>();
    for (size_t jj=0; jj < input_map.dimension(1); ++jj)
      input_map(counter_, jj) = vp->vector[jj];
    counter_++;
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
}

void TensorFlowRepresentation::enqueue(const ProjectionPtr &projection, const Vector &target)
{
  VectorProjection *vp = dynamic_cast<VectorProjection*>(projection.get());
  
  if (vp)
  {
    auto input_map = input_.tensor<float, 2>();
    for (size_t ii=0; ii < input_map.dimension(1); ++ii)
      input_map(counter_, ii) = vp->vector[ii];

    auto target_map = target_.tensor<float, 2>();
    for (size_t ii=0; ii < target_map.dimension(1); ++ii)
      target_map(counter_, ii) = target[ii];

    counter_++;
  }
  else
    throw Exception("representation/tensorflow requires a projector returning a VectorProjection");
}

void TensorFlowRepresentation::read(Matrix *out)
{
  std::vector<std::pair<std::string, tensorflow::Tensor>> inputs = {{input_layer_, input_}};
  
  Tensor learning(tensorflow::DT_BOOL, TensorShape());
  if (!learning_phase_.empty())
  {
    learning.scalar<bool>()() = false;
    inputs.push_back({learning_phase_, learning});
  }
  
  std::vector<Tensor> outputs;
  
  Status run_status = session_->Run(inputs, {output_layer_}, {}, &outputs);
  if (!run_status.ok()) {
    ERROR(run_status.ToString());
    throw Exception("Could not run prediction graph");
  }
  
  // std::swap?
  auto output_map = outputs[0].tensor<float, 2>();
  out->resize(output_map.dimension(0), output_map.dimension(1));
  for (size_t ii=0; ii < output_map.dimension(0); ++ii)
    for (size_t jj=0; jj < output_map.dimension(1); ++jj)
      (*out)(ii, jj) = output_map(ii, jj);
}

void TensorFlowRepresentation::write()
{
  std::vector<std::pair<std::string, tensorflow::Tensor>> inputs = {{input_layer_, input_}, {output_target_, target_}};
  
  Tensor learning(tensorflow::DT_BOOL, TensorShape());
  if (!learning_phase_.empty())
  {
    learning.scalar<bool>()() = true;
    inputs.push_back({learning_phase_, learning});
  }
  
  Tensor weights(tensorflow::DT_FLOAT, TensorShape({target_.shape().dim_size(0)}));
  if (!sample_weights_.empty())
  {
    auto weight_map = weights.tensor<float, 1>();
    for (size_t ii=0; ii < weight_map.dimension(0); ++ii)
      weight_map(ii) = 1.;
    inputs.push_back({sample_weights_, weights});
  }

  // Training
  Status run_status = session_->Run(inputs, {}, {update_node_}, NULL);
  if (!run_status.ok()) {
    ERROR(run_status.ToString());
    throw Exception("Could not run learning graph");
  }
}

size_t TensorFlowRepresentation::size() const
{
  return params_.size();
}

const LargeVector &TensorFlowRepresentation::params() const
{
  std::vector<Tensor> weights;
  
  Status run_status = session_->Run({}, {weights_read_}, {}, &weights);
  if (!run_status.ok()) {
    ERROR(run_status.ToString());
    throw Exception("Could not run weight reading graph");
  }
  
  size_t n = 0;
  weights_shape_.clear();
  for (size_t ii=0; ii < weights.size(); ++ii)
  {
    weights_shape_.push_back(weights[ii].shape());
    n += weights[ii].NumElements();
  }
  
  params_.resize(n);
  
  n = 0;
  for (size_t ii=0; ii < weights.size(); ++ii)
  {
    auto weight_map = weights[ii].flat<float>();
    for (size_t jj=0; jj < weight_map.dimension(0); ++jj)
      params_[n+jj] = weight_map(jj);
    n += weight_map.dimension(0);
  }

  return params_;
}

void TensorFlowRepresentation::setParams(const LargeVector &params)
{
  if (params.size() != params_.size())
  {
    ERROR("Parameter vector size mismatch");
    return;
  }

  params_ = params;
  
  std::vector<std::pair<std::string, tensorflow::Tensor>> weights;
  
  size_t n=0;
  for (size_t ii=0; ii < weights_shape_.size(); ++ii)
  {
    Tensor w = Tensor(tensorflow::DT_FLOAT, weights_shape_[ii]);
    auto w_map = w.flat<float>();
    for (size_t jj=0; jj < w_map.dimension(0); ++jj)
      w_map(jj) = params_[n+jj];
    n += w_map.dimension(0);

    weights.push_back({weights_write_[ii], w});
  }
  
  Status run_status = session_->Run(weights, {}, {weights_node_}, NULL);
  if (!run_status.ok()) {
    ERROR(run_status.ToString());
    throw Exception("Could not run weight writing graph");
  }
}
