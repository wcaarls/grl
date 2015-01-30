/** \file grl.cpp
 * \brief Main GRL sourcefile.
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

#include <grl/configurable.h>
#include <grl/projections/sample.h>
#include <grl/visualization.h>

using namespace grl;

DEFINE_FACTORY(Configurable)

DECLARE_TYPE_NAME(bool)
DECLARE_TYPE_NAME(char)
DECLARE_TYPE_NAME(unsigned char)
DECLARE_TYPE_NAME(short int)
DECLARE_TYPE_NAME(short unsigned int)
DECLARE_TYPE_NAME(int)
DECLARE_TYPE_NAME(unsigned int)
DECLARE_TYPE_NAME(long int)
DECLARE_TYPE_NAME(long unsigned int)
DECLARE_TYPE_NAME(long long int)
DECLARE_TYPE_NAME(long long unsigned int)
DECLARE_TYPE_NAME(float)
DECLARE_TYPE_NAME(double)
DECLARE_TYPE_NAME(Vector)
DECLARE_TYPE_NAME(std::string)

ReadWriteLock SampleStore::rwlock_;

Visualizer *Visualizer::instance_ = NULL;

pthread_once_t RandGen::once_ = PTHREAD_ONCE_INIT;
pthread_mutex_t RandGen::mutex_;
pthread_key_t RandGen::key_;
