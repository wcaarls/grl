/** \file mcts.h
 * \brief Monte-Carlo Tree Search policy header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-06-12
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

#ifndef GRL_MCTS_POLICY_H_
#define GRL_MCTS_POLICY_H_

#include <grl/policy.h>
#include <grl/environments/observation.h>
#include <grl/discretizer.h>

namespace grl
{

class MCTSNode
{
  protected:
    size_t action_, visits_, num_children_;
    uint64_t expanded_actions_;
    double q_, reward_;
    Vector state_;
    bool terminal_;
    MCTSNode *parent_, *children_;
    
  public:
    MCTSNode() : parent_(NULL), children_(NULL) { }
    virtual ~MCTSNode()              { safe_delete_array(&children_); }
  
    size_t action() const            { return action_; }
    double q() const                 { return q_; }
    size_t visits() const            { return visits_; }
    size_t children() const          { return num_children_; } 
    bool expanded(size_t a) const    { return expanded_actions_ & (1<<a); }
    double reward() const            { return reward_; }
    const Vector &state() const      { return state_; }
    bool terminal() const            { return terminal_; }
    MCTSNode *parent()               { return parent_; }
    void orphanize()                 { parent_ = NULL; }

    void init(MCTSNode *parent, size_t action, const Vector &state, double reward, bool terminal)
    {
      action_ = action;
      visits_ = 0;
      num_children_ = 0;
      q_ = 0;
      reward_ = reward;
      state_ = state;
      terminal_ = terminal;
      parent_ = parent;
      expanded_actions_ = 0;
    }
    
    virtual void allocate(size_t max_children)
    {
      children_ = new MCTSNode[max_children];
    }
    
    MCTSNode *expand(size_t action, const Vector &state, double reward, bool terminal)
    {
      expanded_actions_ |= (1<<action);
      children_[num_children_].init(this, action, state, reward, terminal);
      return &children_[num_children_++];
    }
    
    virtual MCTSNode *select(double exploration) const
    {
      // In principle, select() is only called when all children have
      // been expanded, so using num_children_ is proper here.
      if (drand48() < exploration)
        return &children_[lrand48()%num_children_];
      
      double best_value = -std::numeric_limits<double>::infinity();
      size_t best_child = 0;
    
      for (size_t ii=0; ii < num_children_; ++ii)
      {
        double value = children_[ii].q()/children_[ii].visits();
        
        if (value > best_value)
        {
          best_value = value;
          best_child = ii;
        }
      }
      
      return &children_[best_child];
    }
    
    void update(double reward) 
    {
      //std::cout << "Update for node with state " << state() << " with reward " << reward << std::endl;
    
      q_+= reward;
      visits_++;
    }
    
    size_t print(std::ostream &out, size_t maxdepth=0, size_t id=0) const
    {
      size_t lastid=id;
      std::ostringstream oss;
    
      out << "  node_" << id << "[label=\"" << state_ << ", " << q_/visits_ << " (" << visits_ << ")\"];" << std::endl;
      
      if (maxdepth == 1)
        return lastid;
      
      for (size_t ii=0; ii < num_children_; ++ii)
      {
        oss << " node_" << lastid+1;
        out << "  node_" << id << " -> node_" << lastid+1 << " [label=\"" << ii << "\"];" << std::endl;
        lastid = children_[ii].print(out, maxdepth?maxdepth-1:maxdepth, lastid+1);
      }
      
      if (num_children_ > 1)
        out << "  { rank=same; " << oss.str() << " }" << std::endl;
      
      return lastid;
    }
};

// Monte-Carlo Tree Search policy.
class MCTSPolicy : public Policy
{
  public:
    TYPEINFO("mapping/policy/mcts", "Monte-Carlo Tree Search policy")
    
  protected:
    ObservationModel *model_;
    Discretizer *discretizer_;
    
    double gamma_, epsilon_;
    size_t horizon_;
    double budget_;

    // TODO: make act non-const for all policies?  
    mutable MCTSNode *root_, *trunk_;

  public:
    MCTSPolicy() : model_(NULL), discretizer_(NULL), gamma_(1.), epsilon_(0.05), horizon_(100), budget_(0.05), root_(NULL), trunk_(NULL)
    {
    }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual TransitionType act(double time, const Vector &in, Vector *out);
    
  protected:
    virtual void allocate() const
    {
      root_ = new MCTSNode();
    }
    
    Vector select() const
    {
      if (trunk_->children())
        return discretizer_->at(trunk_->state(), trunk_->select(0)->action());
      else
        return discretizer_->at(trunk_->state(), lrand48()%discretizer_->size(trunk_->state()));
    }
    
    MCTSNode *expand(MCTSNode *node) const
    {
      const Vector &state = node->state();
      Vector next;
      double reward;
      int terminal;

      size_t a;
      if (discretizer_->size(state) <= 64)
      {
        // Quasi-pseudorandom action expansion -- not actually pseudorandom
        // except for the first choice.
        a = lrand48()%discretizer_->size(state);
        while (node->expanded(a))
          a = (a + 1)%discretizer_->size(state);
      }
      else
        a = node->children();
        
      Vector action = discretizer_->at(state, a);
          
      model_->step(state, action, &next, &reward, &terminal);
      
      if (!next.size())
      {
        //std::cout << "Failed to expand node due to low confidence" << std::endl;
        return NULL;
      }
      
      if (!node->children())
        node->allocate(discretizer_->size(state));
        
      return node->expand(a, next, reward, terminal);
    }
  
    MCTSNode *treePolicy() const
    {
      MCTSNode *node = trunk_;
      
      for (size_t ii=0; ii < horizon_ && !node->terminal(); ++ii)
      {
        if (node->children() != discretizer_->size(node->state()))
        {
          MCTSNode *child = expand(node);
          
          if (child)
            return child;
          else
            return node;
        }
        else
          node = node->select(epsilon_);
      }
      
      return node;
    }
    
    double defaultPolicy(Vector state, size_t depth) const
    {
      Vector next;
      double reward=0, total_reward=0;
      int terminal=0;
      
      for (size_t ii=0; ii < depth && !terminal; ++ii)
      {
        // Not reentrant
        model_->step(state, discretizer_->at(state, lrand48()%discretizer_->size(state)), &next, &reward, &terminal);
        
        // What to do here?
        if (!next.size())
        {
          //std::cout << "Stopped default policy due to low confidence at step " << ii << " (" << state << ")" << std::endl;
          reward -= 100*(depth-ii);
          break;
        }
        
        state = next;
        total_reward = gamma_*total_reward + reward;
      }
      
      return total_reward;
    }
    
    void print(std::ostream &out, size_t maxdepth=0) const
    {
      out << "digraph mcts_tree {" << std::endl;
      root_->print(out, maxdepth);
      out << "}" << std::endl;
    }
};

class UCTNode : public MCTSNode
{
  public:
    virtual void allocate(size_t max_children)
    {
      children_ = new UCTNode[max_children];
    }

    virtual MCTSNode *select(double exploration) const
    {
      double best_value = -std::numeric_limits<double>::infinity();
      size_t best_child = 0;
      double state_visits=0;
      
      for (size_t ii=0; ii < num_children_; ++ii)
        state_visits += children_[ii].visits();
      state_visits = std::log(state_visits);
      
      for (size_t ii=0; ii < num_children_; ++ii)
      {
        double q = children_[ii].q(), action_visits = children_[ii].visits();
        double value = q/action_visits + 2*exploration*sqrt(state_visits/action_visits);
        
        if (value > best_value)
        {
          best_value = value;
          best_child = ii;
        }
      }
      
      return &children_[best_child];
    }
};

// Monte-Carlo Tree Search policy using UCB1 action selection.
class UCTPolicy : public MCTSPolicy
{
  public:
    TYPEINFO("policy/uct", "Monte-Carlo Tree Search policy using UCB1 action selection")
    
  protected:
    virtual void allocate() const
    {
      root_ = new UCTNode();
    }
};

}

#endif /* GRL_MCTS_POLICY_H__ */
