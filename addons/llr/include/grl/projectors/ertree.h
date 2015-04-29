/** \file ertree.h
 * \brief Extremely Randomized Trees projector header file.
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

#ifndef GRL_ERTREE_PROJECTOR_H_
#define GRL_ERTREE_PROJECTOR_H_

#include <stdint.h>

#include <grl/projector.h>
#include <grl/projections/sample.h>

namespace grl {

class ERTreeNode
{
  protected:
    class ERTree *tree_;
    size_t *samples_;
    size_t num_samples_;
  
    uint8_t split_attribute_;
    double split_point_;
  
    ERTreeNode *left_, *right_;
  
  public:
    ERTreeNode(ERTree *tree, size_t *samples, size_t num_samples, Vector var);
    ~ERTreeNode()
    {
      safe_delete(&left_);
      safe_delete(&right_);
    }

    std::vector<size_t> read(const Vector &input) const;
};

class ERTree
{
  protected:
    StorePtr store_;
    size_t *samples_;
    ERTreeNode *root_;
    
    size_t inputs_, outputs_, splits_, leaf_size_;
    
  public:
    ERTree(StorePtr store, size_t inputs, size_t outputs, size_t splits, size_t leaf_size, Vector var) :
      store_(store), samples_(NULL), root_(NULL), inputs_(inputs), outputs_(outputs), splits_(splits), leaf_size_(leaf_size)
    {
      samples_ = new size_t[store->size()];
      for (size_t ii=0; ii < store->size(); ++ii)
        samples_[ii] = ii;
        
      root_ = new ERTreeNode(this, samples_, store->size(), var);
    }
    
    ~ERTree()
    {
      safe_delete_array(&samples_);
      safe_delete(&root_);
    }

    StorePtr store() { return store_; }    
    size_t inputs() const { return inputs_; }
    size_t outputs() const { return outputs_; }
    size_t splits() const { return splits_; }
    size_t leafSize() const { return leaf_size_; }
    
    std::vector<size_t> read(const Vector &input) const
    {
      return root_->read(input);
    }

    static Vector variance(StorePtr storeptr, size_t outputs);
};

/// Projects onto samples found through the Extra-trees algorithm by Geurts et al.
class ERTreeProjector : public SampleProjector
{
  public:
    TYPEINFO("projector/sample/ertree")
    
  public:
    StorePtr store_;
    ERTree **forest_;
    
    size_t max_samples_, trees_, splits_, leaf_size_, inputs_, outputs_, indexed_samples_;
    mutable ReadWriteLock rwlock_;

  public:
    ERTreeProjector() : forest_(NULL), max_samples_(100000), trees_(20), splits_(5), leaf_size_(10), inputs_(1), outputs_(1), indexed_samples_(0) { }
    ~ERTreeProjector()
    {
      if (forest_)
      {
        for (size_t ii=0; ii < trees_; ++ii)
          safe_delete(&forest_[ii]);
        safe_delete_array(&forest_);
      }
    }
    
    void reindex();
    
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From SampleProjector
    virtual ERTreeProjector *clone() const;
    virtual void push(Sample *sample);
    virtual StorePtr store() { ReadGuard guard(rwlock_); return store_; }
    virtual ProjectionPtr project(const Vector &in) const;
    virtual void finalize();
};

}

#endif /* GRL_ERTREE_PROJECTOR_H_ */
