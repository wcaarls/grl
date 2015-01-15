/*
 * trace.h
 *
 *  Created on: Jul 30, 2014
 *      Author: wcaarls
 */

#ifndef TRACE_H_
#define TRACE_H_

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

        virtual const ProjectionPtr projection() const = 0;
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

    virtual size_t size() const = 0;
    virtual void add(ProjectionPtr projection, double decay=1.0) = 0;
    virtual const ProjectionPtr back() const = 0;
    virtual void clear() = 0;

    iterator begin() const { return iterator(*this); }
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

    std::vector<ProjectionDecay> projections_;

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
          return trace_.projections_[trace_.projections_.size()-index()-1].projection;
        }

        virtual double weight() const
        {
          return weight_;
        }

      protected:
        virtual void inc()
        {
          weight_ *= trace_.projections_[trace_.projections_.size()-index()-1].decay;
          TraceIteratorImpl::inc();
        }
    };

  protected:
    virtual TraceIteratorImpl *createIterator(iterator &it) const
    {
      return new EnumeratedTraceIteratorImpl(*this, it);
    }

  public:
    virtual size_t size() const { return projections_.size(); }
    virtual void add(ProjectionPtr projection, double decay=1.0) = 0;
    virtual const ProjectionPtr back() const
    {
      if (size())
        return projections_[size()-1].projection;
      else
        return ProjectionPtr();
    }
    virtual void clear()
    {
      projections_.clear();
    }
};

/// Replacing eligibility Trace.
class ReplacingEnumeratedTrace : public EnumeratedTrace
{
  public:
    TYPEINFO("trace/enumerated/replacing")

  virtual ReplacingEnumeratedTrace *clone() const
  {
    ReplacingEnumeratedTrace *trace = new ReplacingEnumeratedTrace();
    for (size_t ii=0; ii < size(); ++ii)
      trace->projections_.push_back(projections_[ii].clone());
    return trace;
  }

  virtual void add(ProjectionPtr projection, double decay=1.0)
  {
    for (size_t ii=0; ii < size(); ++ii)
      projections_[ii].projection->ssub(*projection);

    projections_.push_back(ProjectionDecay(projection, decay));
  }
};

/// Accumulating eligibility Trace.
class AccumulatingEnumeratedTrace : public EnumeratedTrace
{
  public:
    TYPEINFO("trace/enumerated/accumulating")

  virtual AccumulatingEnumeratedTrace *clone() const
  {
    AccumulatingEnumeratedTrace *trace = new AccumulatingEnumeratedTrace();
    for (size_t ii=0; ii < size(); ++ii)
      trace->projections_.push_back(projections_[ii].clone());
    return trace;
  }

  virtual void add(ProjectionPtr projection, double decay=1.0)
  {
    projections_.push_back(ProjectionDecay(projection, decay));
  }
};

}

#endif /* TRACE_H_ */
