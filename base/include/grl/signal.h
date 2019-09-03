/** \file signal.h
 * \brief Inter-component signalling.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-15
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

#ifndef GRL_SIGNAL_H_
#define GRL_SIGNAL_H_

#include <itc/queue.h>

#include <grl/configurable.h>
#include <grl/mutex.h>

namespace grl
{

template<class T>
class Signal : public Configurable
{
  protected:
    itc::Queue<T> queue_;
    itc::QueueWriter<T> writer_;
    itc::QueueReader<T> reader_;

  public:
    Signal()
    {
      writer_ = queue_.getWriter();
      reader_ = queue_.addReader();
      reader_.disengage();
    }
    
    itc::QueueReader<T> addReader()
    {
      return queue_.addReader();
    }
  
    /// Gets last written value, or empty element if no value has been written.
    virtual T get()
    {
      reader_.reengage();
      T element;
      if (reader_.test())
        element = reader_.get();
      reader_.disengage();
      return element;
    }
    
    /// Sets new value.    
    virtual void set(const T &state)
    {
      writer_.write(state);
    }
};

/// Encapsulates a vector (e.g. system state).
class VectorSignal : public Signal<Vector> // Do we need to have large vector here?
{
  public:
    TYPEINFO("signal/vector", "Vector-based signal (state, observation, etc.)");
};

/// Encapsulates a matrix (e.g. trajectory).
class MatrixSignal : public Signal<Matrix>
{
  public:
    TYPEINFO("signal/matrix", "Matrix-based signal (trajectory, etc.)");
};

}

#endif /* GRL_SIGNAL_H_ */
