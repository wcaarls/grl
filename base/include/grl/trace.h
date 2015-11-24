/** \file trace.h
 * \brief Generic and basic trace definitions.
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

#ifndef GRL_TRACE_H_
#define GRL_TRACE_H_

#include <deque>
#include <grl/mutex.h>
#include <grl/configurable.h>
#include <grl/projection.h>

namespace grl
{

/// Stores a trace of Projection%s.
/**
 * May also integrate projections into a single one, if that is more convenient.
 */
class Trace : public Configurable
{
  public:
    class iterator;

    /// Inherit from this to define your own Trace::iterator.
    class TraceIteratorImpl
    {
      friend class iterator;

      protected:
        iterator &it_;

      public:
        TraceIteratorImpl(iterator &it) : it_(it) { }
        virtual ~TraceIteratorImpl() { }

        /// Projection.
        virtual const ProjectionPtr projection() const = 0;
        
        /// Eligibility.
        virtual double weight() const = 0;

      protected:
        size_t index() const;
        virtual void inc();
    };

    /// Basic iterator for eligibility Trace%s. Delegates access to TraceIteratorImpl.
    class iterator
    {
      friend class TraceIteratorImpl;

      protected:
        size_t index_;
        TraceIteratorImpl *impl_;

      public:
        iterator(size_t index) : index_(index), impl_(NULL) { }
        iterator(const Trace &trace) : index_(0)
        {
          impl_ = trace.createIterator(*this);
        }

        ~iterator()
        {
          if (impl_)
            delete impl_;
        }

        bool operator==(const iterator &rhs) const { return index_ == rhs.index_; }
        bool operator!=(const iterator &rhs) const { return index_ != rhs.index_; }
        iterator &operator++() { impl_->inc(); return *this; }
        TraceIteratorImpl *operator->() { return impl_; }
    };

  protected:
    virtual TraceIteratorImpl *createIterator(iterator &it) const = 0;

  public:
    virtual ~Trace() { }
    virtual Trace *clone() const = 0;

    /// Returns trace length.
    virtual size_t size() const = 0;
    
    /// Adds a projection to the trace.
    virtual void add(ProjectionPtr projection, double decay=1.0) = 0;
    
    /// Returns most recently added projection.
    virtual const ProjectionPtr back() const = 0;
    
    /// Clear the trace.
    virtual void clear() = 0;

    /// Returns iterator pointing to start of trace.
    iterator begin() const { return iterator(*this); }
    
    /// Returns iterator pointing to beyond the end of the trace.
    iterator end() const { return iterator(size()); }
};

inline size_t Trace::TraceIteratorImpl::index() const
{
  return it_.index_;
}

inline void Trace::TraceIteratorImpl::inc()
{
  it_.index_++;
}

/// Trace that keeps a history of ProjectionPtr%s.
class EnumeratedTrace : public Trace
{
  protected:
    /// A (ProjectionPtr, double) pair to store trace weights.
    struct ProjectionDecay
    {
      ProjectionPtr projection;
      double decay;

      ProjectionDecay(ProjectionPtr _projection, double _decay) : projection(_projection), decay(_decay) { }

      ProjectionDecay clone() const
      {
        return ProjectionDecay(ProjectionPtr(projection->clone()), decay);
      }
    };

    mutable Instance<std::deque<ProjectionDecay> > projections_;

  public:
    /// EnumeratedTrace iterator.
    class EnumeratedTraceIteratorImpl : public TraceIteratorImpl
    {
      protected:
        const EnumeratedTrace &trace_;
        double weight_;

      public:
        EnumeratedTraceIteratorImpl(const EnumeratedTrace &trace, iterator &it) : TraceIteratorImpl(it), trace_(trace), weight_(1.) { }

        virtual const ProjectionPtr projection() const
        {
          return (*trace_.projections_)[trace_.projections_->size()-index()-1].projection;
        }

        virtual double weight() const
        {
          return weight_;
        }

      protected:
        virtual void inc()
        {
          weight_ *= (*trace_.projections_)[trace_.projections_->size()-index()-1].decay;
          TraceIteratorImpl::inc();
        }
    };

  protected:
    virtual TraceIteratorImpl *createIterator(iterator &it) const
    {
      return new EnumeratedTraceIteratorImpl(*this, it);
    }

  public:
    virtual size_t size() const { return projections_->size(); }
    virtual void add(ProjectionPtr projection, double decay=1.0) = 0;
    virtual const ProjectionPtr back() const
    {
      if (size())
        return (*projections_)[size()-1].projection;
      else
        return ProjectionPtr();
    }
    virtual void clear()
    {
      projections_->clear();
    }
};

/// Replacing eligibility Trace.
class ReplacingEnumeratedTrace : public EnumeratedTrace
{
  public:
    TYPEINFO("trace/enumerated/replacing", "Replacing eligibility trace using a queue of projections")
    
  protected:
    double total_decay_;
    
  public:
    ReplacingEnumeratedTrace() : total_decay_(1.0) { }
  
    virtual ReplacingEnumeratedTrace *clone() const
    {
      return new ReplacingEnumeratedTrace();
   }

    virtual void add(ProjectionPtr projection, double decay=1.0)
    {
      std::deque<ProjectionDecay> *p = projections_.instance();
    
      if (decay < 0.01)
      {
        p->clear();
        total_decay_ = 1.0;
      }
    
      for (size_t ii=0; ii < size(); ++ii)
        (*p)[ii].projection->ssub(*projection);

      p->push_back(ProjectionDecay(projection, decay));
      total_decay_ *= decay;
      
      while (total_decay_ < 0.01 && p->size() > 1)
      {
        total_decay_ /= p->front().decay;
        p->pop_front();
      }
    }
};

/// Accumulating eligibility Trace.
class AccumulatingEnumeratedTrace : public EnumeratedTrace
{
  public:
    TYPEINFO("trace/enumerated/accumulating", "Accumulating eligibility trace using a queue of projections")

  protected:
    double total_decay_;
    
  public:
    AccumulatingEnumeratedTrace() : total_decay_(1.0) { }
  
    virtual AccumulatingEnumeratedTrace *clone() const
    {
      return new AccumulatingEnumeratedTrace();
    }

    virtual void add(ProjectionPtr projection, double decay=1.0)
    {
      std::deque<ProjectionDecay> *p = projections_.instance();
    
      if (decay < 0.01)
      {
        p->clear();
        total_decay_ = 1.0;
      }
    
      p->push_back(ProjectionDecay(projection, decay));
      total_decay_ *= decay;
      
      while (total_decay_ < 0.01 && p->size() > 1)
      {
        total_decay_ /= p->front().decay;
        p->pop_front();
      }
    }
};

}

#endif /* GRL_TRACE_H_ */
