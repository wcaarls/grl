/** \file converter.h
 * \brief Class which is capable of remapping states and actions.
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
 
#ifndef GRL_CONVERTER_H_
#define GRL_CONVERTER_H_

#include <grl/configurable.h>

namespace grl
{

/// Converts signals.
class StateActionConverter : public Configurable
{
  public:
    TYPEINFO("converter/state_action_converter", "Configurable which is capable of remapping states and actions")

  protected:
    std::vector<int> state_map_, action_map_;
    int state_in_size_, action_out_size_;
  public:
    StateActionConverter() : state_in_size_(0), action_out_size_(0) { }
    virtual ~StateActionConverter() { }

    // From Configurable
    virtual void request(ConfigurationRequest *config)
    {
      config->push_back(CRP("state_in", "string.state_in", "Comma-separated list of state elements in the input vector"));
      config->push_back(CRP("state_out", "string.state_out", "Comma-separated list of state elements in the output vector"));
      config->push_back(CRP("action_in", "string.action_in", "Comma-separated list of action elements observed in the input vector"));
      config->push_back(CRP("action_out", "string.action_out", "Comma-separated list of action elements provided in the output vector"));
    }

    virtual void configure(Configuration &config)
    {
      const std::vector<std::string> state_in = cutLongStr( config["state_in"].str() );
      const std::vector<std::string> state_out = cutLongStr( config["state_out"].str() );
      const std::vector<std::string> action_in = cutLongStr( config["action_in"].str() );
      const std::vector<std::string> action_out = cutLongStr( config["action_out"].str() );

      state_in_size_ = state_in.size();
      action_out_size_ = action_out.size();

      prepare(state_in, state_out, state_map_);
      prepare(action_in, action_out, action_map_);

      INFO("State map: " << state_map_);
      INFO("Action map: " << action_map_);
    }

    virtual void reconfigure(const Configuration &config) { }

    // Own
    virtual int convert(const Vector &state_in, Vector &state_out, const Vector &action_in, Vector &action_out) const
    {
      convert_state(state_in, state_out);
      convert_action(action_in, action_out);
    }

    virtual int convert_state(const Vector &state_in, Vector &state_out) const
    {
      if (state_out.size() < state_map_.size())
        state_out.resize(state_map_.size());

      if (state_map_.size() > 0)
      {
        for (int i = 0; i < state_map_.size(); i++)
          state_out[i] = state_in[ state_map_[i] ];
      }
      else
      {
        state_out = state_in;
      }
    }

    virtual int convert_action(const Vector &action_in, Vector &action_out) const
    {
      if (action_out.size() < action_map_.size())
        action_out.resize(action_map_.size());

      if (action_map_.size() > 0)
      {
        for (int i = 0; i < action_map_.size(); i++)
          action_out[i] = action_in[ action_map_[i] ];
        //std::cout << action_in << " > " << action_out << std::endl;
      }
      else
      {
        action_out = action_in;
      }
    }

    virtual void prepare(const std::vector<std::string> in, const std::vector<std::string> out, std::vector<int> &map) const
    {
      std::vector<std::string>::const_iterator it_out = out.begin();
      std::vector<std::string>::const_iterator it_in = in.begin();
      for (; it_out < out.end(); it_out++)
      {
        bool found = false;
        it_in = in.begin();
        for (int i = 0; it_in < in.end(); it_in++, i++)
        {
          if (*it_out == *it_in)
          {
            TRACE("Adding to the observation vector (physical state): " << *it_out);
            map.push_back(i);
            found = true;
            break;
          }
        }

        if (!found)
          throw Exception("Field '" + *it_out + "' is not matched with any input");
      }
    }

    virtual int get_state_in_size() { return state_in_size_; }
    virtual int get_action_out_size() { return action_out_size_; }
};

REGISTER_CONFIGURABLE(StateActionConverter)

}

#endif /* GRL_CONVERTER_H_ */
