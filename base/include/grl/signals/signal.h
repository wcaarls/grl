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

#ifndef GRL_SIGNAL_DEF_H_
#define GRL_SIGNAL_DEF_H_

#include <itc/itc.h>

#include <grl/configurable.h>
#include <grl/mutex.h>

namespace grl
{

enum EnvSignal {sigEnvNone, sigEnvInit};

/// Encapsulates a vector (e.g. system state).
class VectorSignal : public Configurable
{
  public:
    TYPEINFO("signal/vector", "Vector-based signal (state, observation, etc.)");
    
  protected:
    itc::SharedVariable<LargeVector> var_;

  public:
    /// Returns current value.
    virtual LargeVector get()
    {
      return var_.get();
    }

    /// Sets new value.    
    virtual void set(const LargeVector &state)
    {
      var_.write(state);
    }

    /// Returns true if the value has changed.
    virtual bool test()
    {
      return var_.test();
    }

    /// Reads a new value, waiting until it changes if necessary.    
    virtual LargeVector read()
    {
      return var_.read();
    }
};

/// Encapsulates a matrix (e.g. trajectory).
class MatrixSignal : public Configurable
{
  public:
    TYPEINFO("signal/matrix", "Matrix-based signal (trajectory, etc.)");
    
  protected:
    itc::SharedVariable<Matrix> var_;

  public:
    /// Returns current value.
    virtual Matrix get()
    {
      return var_.get();
    }

    /// Sets new value.    
    virtual void set(const Matrix &trajectory)
    {
      var_.write(trajectory);
    }

    /// Returns true if the value has changed.
    virtual bool test()
    {
      return var_.test();
    }

    /// Reads a new value, waiting until it changes if necessary.    
    virtual Matrix read()
    {
      return var_.read();
    }
};

}

#endif /* GRL_SIGNAL_DEF_H_ */