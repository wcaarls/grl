/** \file leo_preprogrammed.cpp
 * \brief Leo preprogrammed agent source file.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-07-25
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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

#include <grl/agents/leo_preprogrammed.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoPreprogrammedAgent)

void LeoPreprogrammedAgent::request(ConfigurationRequest *config)
{

}

void LeoPreprogrammedAgent::configure(Configuration &config)
{

}

void LeoPreprogrammedAgent::reconfigure(const Configuration &config)
{
}

LeoPreprogrammedAgent *LeoPreprogrammedAgent::clone() const
{
  LeoPreprogrammedAgent *agent = new LeoPreprogrammedAgent();
  return agent;
}

void LeoPreprogrammedAgent::start(const Vector &obs, Vector *action)
{
  time_ = 0.;

}

void LeoPreprogrammedAgent::step(double tau, const Vector &obs, double reward, Vector *action)
{
  time_ += tau;

}

void LeoPreprogrammedAgent::end(double tau, const Vector &obs, double reward)
{
}
