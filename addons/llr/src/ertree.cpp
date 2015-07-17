/** \file ertree.cpp
 * \brief Extremely randomized trees projector source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-03-27
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

#include <string.h>

#include <grl/projectors/ertree.h>

using namespace grl;

REGISTER_CONFIGURABLE(ERTreeProjector)

ERTreeNode::ERTreeNode(ERTree *tree, size_t *samples, size_t num_samples, Vector var) :
  samples_(samples), num_samples_(num_samples), split_attribute_(255), split_point_(0), left_(NULL), right_(NULL)
{
  SampleStore &store = *tree->store().get();
  
  // Don't split with too few samples
  if (num_samples_ < tree->leafSize())
    return;
    
  // Don't split if outputs have no variance
  if (sum(var) < 0.00001)
    return;

  // Find minimum and maximum attribute values
  Vector min(tree->inputs()), max(tree->inputs());
  
  for (size_t jj=0; jj < tree->inputs(); ++jj)
    max[jj] = min[jj] = store[samples_[0]]->in[jj];

  for (size_t ii=1; ii < num_samples_; ++ii)
    for (size_t jj=0; jj < tree->inputs(); ++jj)
    {
      min[jj] = fmin(min[jj], store[samples_[ii]]->in[jj]);
      max[jj] = fmax(max[jj], store[samples_[ii]]->in[jj]);
    }
    
  // Don't split if inputs have no variance
  bool input_variance = false;
  for (size_t jj=0; jj < tree->inputs(); ++jj)
    if (min[jj] != max[jj])
      input_variance = true;

  if (!input_variance)
    return;
    
  // Find best split
  double best_score = -std::numeric_limits<double>::infinity();
  size_t best_samples_l = 0, best_samples_r = 0;
  Vector best_var_l, best_var_r;

  for (size_t kk=0; kk < tree->splits(); ++kk)
  {
    // Choose random split
    uint8_t attr = RandGen::getInteger(tree->inputs());
    double point = RandGen::getUniform(min[attr], max[attr]);
    
    // Calculate means
    Vector mean_l(tree->outputs(), 0.), mean_r = mean_l;
    size_t samples_l=0, samples_r=0;
    
    for (size_t ii=0; ii < num_samples_; ++ii)
    {
      if (store[samples_[ii]]->in[attr] < point)
      {
        samples_l++;
        for (size_t jj=0; jj < tree->outputs(); ++jj)
          mean_l[jj] += store[samples_[ii]]->out[jj];
      }
      else
      {
        samples_r++;
        for (size_t jj=0; jj < tree->outputs(); ++jj)
          mean_r[jj] += store[samples_[ii]]->out[jj];
      }
    }
    
    // Don't accept trivial splits
    if (!samples_l || !samples_r)
    {
      --kk;
      continue;
    }
     
    for (size_t jj=0; jj < tree->outputs(); ++jj)
    {
      mean_l[jj] /= samples_l;
      mean_r[jj] /= samples_r;
    }
    
    // Calculate variances
    Vector var_l(tree->outputs(), 0.), var_r = var;
    
    for (size_t ii=0; ii < num_samples_; ++ii)
    {
      if (store[samples_[ii]]->in[attr] < point)
        for (size_t jj=0; jj < tree->outputs(); ++jj)
          var_l[jj] += pow(store[samples_[ii]]->out[jj] - mean_l[jj], 2);
      else
        for (size_t jj=0; jj < tree->outputs(); ++jj)
          var_r[jj] += pow(store[samples_[ii]]->out[jj] - mean_r[jj], 2);
    }
    
    for (size_t jj=0; jj < tree->outputs(); ++jj)
    {
      var_l[jj] /= samples_l;
      var_r[jj] /= samples_r;
    }
    
    // Calculate score
    double score=0;
    
    for (size_t jj=0; jj < tree->outputs(); ++jj)
      if (var[jj] > 0)
        score += (var[jj] - ((double)samples_l)/num_samples_*var_l[jj] - ((double)samples_r)/num_samples_*var_r[jj])/var[jj];
        
    // Remember best score
    if (score > best_score)
    {
      split_attribute_ = attr;
      split_point_ = point;
      best_score = score;
      best_samples_l = samples_l;
      best_samples_r = samples_r;
      best_var_l = var_l;
      best_var_r = var_r;
    }
  }
  
  if (split_attribute_ == 255)
  {
    ERROR("Could not find a split");
    return;
  }
  
  // Implement chosen split
  size_t *new_samples = new size_t[num_samples_];
  
  size_t idx_l = 0;
  size_t idx_r = best_samples_l;
  for (size_t ii=0; ii < num_samples_; ++ii)
    if (store[samples_[ii]]->in[split_attribute_] < split_point_)
      new_samples[idx_l++] = samples_[ii];
    else
      new_samples[idx_r++] = samples_[ii];

  // Copy into original sample map
  memcpy(samples_, new_samples, num_samples_*sizeof(size_t));
  delete[] new_samples;  

  // Recurse
  left_ = new ERTreeNode(tree, samples_, best_samples_l, best_var_l);
  right_ = new ERTreeNode(tree, &samples_[best_samples_l], best_samples_r, best_var_r);
}

std::vector<size_t> ERTreeNode::read(const Vector &input) const
{
  if (split_attribute_ == 255)
  {
    // Leaf node
    std::vector<size_t> ret(num_samples_);
    memcpy(ret.data(), samples_, num_samples_*sizeof(size_t));
    return ret;
  }
  else if (input[split_attribute_] < split_point_)
    return left_->read(input);
  else
    return right_->read(input);
}

Vector ERTree::variance(StorePtr storeptr, size_t outputs)
{
  SampleStore &store = *storeptr.get();
  
  // Calculate means
  Vector mean(outputs, 0.);
  
  for (size_t ii=0; ii < store.size(); ++ii)
    for (size_t jj=0; jj < outputs; ++jj)
      mean[jj] += store[ii]->out[jj];

  for (size_t jj=0; jj < outputs; ++jj)
    mean[jj] /= store.size();
  
  // Calculate variances
  Vector var(outputs, 0.);
  
  for (size_t ii=0; ii < store.size(); ++ii)
    for (size_t jj=0; jj < outputs; ++jj)
      var[jj] += pow(store[ii]->out[jj] - mean[jj], 2);
  
  for (size_t jj=0; jj < outputs; ++jj)
    var[jj] /= store.size();
    
  return var;
}

void ERTreeProjector::request(const std::string &role, ConfigurationRequest *config)
{
  config->push_back(CRP("samples", "Maximum number of samples to store", max_samples_, CRP::Configuration, 100));
  config->push_back(CRP("trees", "Number of trees in the forest", trees_, CRP::Configuration, 1));
  config->push_back(CRP("splits", "Number of candidate splits", splits_, CRP::Configuration, 1));
  config->push_back(CRP("leaf_size", "Maximum number of samples in a leaf", leaf_size_, CRP::Configuration, 1));
  
  if (role == "observation")
    config->push_back(CRP("inputs", "int.observation_dims", "Number of input dimensions", inputs_, CRP::System, 1, SS_MAX_COORDS));
  else if (role == "action")
    config->push_back(CRP("inputs", "int.action_dims", "Number of input dimensions", inputs_, CRP::System, 1, SS_MAX_COORDS));
  else if (role == "pair")
    config->push_back(CRP("inputs", "int.observation_dims+int.action_dims", "Number of input dimensions", inputs_, CRP::System, 1, SS_MAX_COORDS));
  else
    config->push_back(CRP("inputs", "Number of input dimensions", inputs_, CRP::System, 1, SS_MAX_COORDS));

  config->push_back(CRP("outputs", "Number of output dimensions", outputs_, CRP::System, 1, SS_MAX_COORDS));
}

void ERTreeProjector::configure(Configuration &config)
{
  max_samples_ = config["samples"];
  trees_ = config["trees"];
  splits_ = config["splits"];
  leaf_size_ = config["leaf_size"];
  inputs_ = config["inputs"];
  outputs_ = config["outputs"];

  // Initialize memory
  reset();
}

void ERTreeProjector::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
  {
    DEBUG("Initializing sample store");
    
    {
      WriteGuard guard(rwlock_);
      store_ = StorePtr(new SampleStore());
      if (forest_)
      {
        for (size_t ii=0; ii < trees_; ++ii)
          safe_delete(&forest_[ii]);
        safe_delete_array(&forest_);
      }
      indexed_samples_ = 0;
    }
  }
}

void ERTreeProjector::push(Sample *sample)
{
  WriteGuard guard(rwlock_);
  
  store_->push_back(sample);
}

/**
 * @brief Actual build of trees happen here
 */
void ERTreeProjector::reindex()
{
  // Create new pruned store
  StorePtr newstore = StorePtr(store_->prune(max_samples_));
  
  DEBUG("Building new forest with " << newstore->size() << " samples");

  ERTree **newforest = new ERTree*[trees_];
  
  // Precompute variance for entire dataset
  Vector var = ERTree::variance(newstore, outputs_);

  // Build new forest using new store
  #ifdef _OPENMP
  #pragma omp parallel for default(shared)
  #endif
  for (size_t ii=0; ii < trees_; ++ii)
    newforest[ii] = new ERTree(newstore, inputs_, outputs_, splits_, leaf_size_, var);
  
  {
    WriteGuard guard(rwlock_);

    store_ = newstore;
    if (forest_)
    {
      for (size_t ii=0; ii < trees_; ++ii)
        safe_delete(&forest_[ii]);
      safe_delete_array(&forest_);
    }
    forest_ = newforest;
    indexed_samples_ = store_->size();
  }
}

ERTreeProjector *ERTreeProjector::clone() const
{
  return NULL;
}

/**
 * @brief Find idexies of neighbours of the query
 * @param in - query point
 * @return If there are constructed trees, then the function returnes idexies of neighbours of the query point
 */
ProjectionPtr ERTreeProjector::project(const Vector &in) const
{
  ReadGuard guard(rwlock_);
  
  // Create projection  
  SampleProjection *projection = new SampleProjection;
  projection->store = store_;
  projection->query = in;
  
  if (!forest_)
    return ProjectionPtr(projection);

  if (in.size() != inputs_)
    throw bad_param("projector/sample/ertree:dims");
    
  // Find neighboring samples in leaf nodes
  std::vector<size_t> neighbors;
  
  for (size_t ii=0; ii < trees_; ++ii)
  {
    std::vector<size_t> tree_neighbors = forest_[ii]->read(in);
    neighbors.insert(neighbors.end(), tree_neighbors.begin(), tree_neighbors.end());
  }
  
  // Fill projection
  projection->indices = neighbors;
  projection->weights.resize(neighbors.size(), 1.);

  return ProjectionPtr(projection);
}

void ERTreeProjector::finalize()
{
  reindex();
}
