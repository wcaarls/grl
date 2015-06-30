/** \file export_csv.cpp
 * \brief Export observations, actions, rewards or any combination of those.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@gmail.com>
 * \date      2015-06-29
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

#include <grl/log/export_csv.h>

using namespace grl;

bool ExportCSV::start(std::string file_name, size_t obs_dim, size_t action_dim, size_t reward_dim)
{
  obs_dim_ = obs_dim;
  action_dim_ = action_dim;
  reward_dim_ = reward_dim;

  if (!(obs_dim_+action_dim_+reward_dim_))
  {
    std::cout << "Nothing to export in CSV" << std::endl;
    return false;
  }

  ofs.open(file_name.c_str(), std::ios_base::out);
  if (!ofs.is_open())
  {
    std::cout << "Error opening log file " << file_name << std::endl;
    return false;
  }

  // CSV Header
  std::ostringstream oss;
  oss << "#GRL CSV export" << std::endl << "NAME:" << std::endl << file_name << std::endl << "COLUMNS:" << std::endl << "Time";
  for (size_t i = 0; i < obs_dim_; i++)
    oss << ", x[" << i << "]";
  for (size_t i = 0; i < action_dim_; i++)
    oss << ", u[" << i << "]";
  if (reward_dim_ == 1)
    oss << ", reward";
  oss << std::endl << "DATA:" << std::endl;
  ofs << oss.str();

  time_ = 0;
  return ofs.is_open();
}

void ExportCSV::stop()
{
  if (ofs.is_open())
    ofs.close();
}

void ExportCSV::log(double tau, Vector obs, Vector action, double reward)
{
  if (!ofs.is_open())
    return;

  time_ += tau;

  std::ostringstream oss;
  oss << std::setw(6) << time_;
  for (size_t i = 0; i < obs_dim_; i++)
    oss << std::setw(15) << obs[i];
  for (size_t i = 0; i < action_dim_; i++)
    oss << std::setw(15) << action[i];
  if (reward_dim_ == 1)
    oss << std::setw(15) << reward;

  if (ofs.is_open())
    ofs << oss.str() << std::endl;
}
