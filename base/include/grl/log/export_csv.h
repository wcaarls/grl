/** \file export_csv.h
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

#ifndef GRL_EXPORT_CSV_H_
#define GRL_EXPORT_CSV_H_
#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>

#include <grl/vector.h>

namespace grl
{

//#define LOG_OBS     0x1
//#define LOG_ACTIONS 0x2
//#define LOG_REWARDS 0x4

class ExportCSV
{
private:
  std::ofstream ofs;
  size_t obs_dim_, action_dim_, reward_dim_;
  double time_;

public:
  ExportCSV() {}
  ~ExportCSV() {}
  bool start(std::string file_name, size_t obs_dim, size_t action_dim, size_t reward_dim);
  void log(double tau, Vector obs, Vector action, double reward);
  void stop();
};

}

#endif

