/** \file predictor.h
 * \brief Generic predictor definition.
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

#ifndef GRL_PREDICTOR_H_
#define GRL_PREDICTOR_H_

#include <grl/grl.h>
#include <grl/configurable.h>
#include <grl/importer.h>
#include <grl/exporter.h>

namespace grl
{

/// Estimates a function from Transition%s.
class Predictor : public Configurable
{
  protected:
    Importer *importer_;
    Exporter *exporter_;

  public:
    Predictor() : importer_(NULL), exporter_(NULL) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    /// Update the estimation.
    virtual void update(const Transition &transition);

    /// Update the estimation for a batch of transitions.
    /// NOTE: these are not necessarily part of the same trajectory.
    virtual void update(const std::vector<const Transition*> &transitions)
    {
      for (size_t ii=0; ii < transitions.size(); ++ii)
      {
        update(*transitions[ii]);
        finalize();
      }
    }
    
    /// Signal completion of an episode.
    virtual void finalize();
};

class CriticPredictor : public Predictor
{
  public:
    virtual void update(const Transition &transition)
    {
      criticize(transition, Action());
    }

    virtual void update(const std::vector<const Transition*> &transitions)
    {
      criticize(transitions, std::vector<const Action*>());
    }
    
    /// Update the estimation. Returns reinforcement signal for actor.
    virtual double criticize(const Transition &transition, const Action &action) = 0;

    /// Update the estimation for a batch of transitions.
    /// NOTE: these are not necessarily part of the same trajectory.
    virtual LargeVector criticize(const std::vector<const Transition*> &transitions, const std::vector<const Action*> &actions)
    {
      LargeVector critique(transitions.size());
      
      if (actions.size())
      {
        for (size_t ii=0; ii < transitions.size(); ++ii)
        {
          critique[ii] = criticize(*transitions[ii], *actions[ii]);
          finalize();
        }
      }
      else
      {
        for (size_t ii=0; ii < transitions.size(); ++ii)
        {
          criticize(*transitions[ii], Action());
          finalize();
        }
      }
      
      return critique;
    }
};

}

#endif /* GRL_PREDICTOR_H_ */
