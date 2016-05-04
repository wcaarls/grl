/** \file compat.h
 * \brief Cross-platform compatibility definitions
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-09-03
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

#ifndef GRL_COMPAT_H_
#define GRL_COMPAT_H_

#include <iostream>

#ifdef WIN32
#include <ctime>
struct drand48_data { unsigned char dummy; };
#define srand48_r(x, y) srand(x)
#define drand48_r(x, y) do { *(y) = rand()/(RAND_MAX+1.); } while (0)
#define lrand48() (rand()%RAND_MAX)
void __stdcall Sleep(_In_ unsigned int dwMilliseconds);
#define usleep(x) Sleep(((unsigned int)(x))/1000)
inline int round( double r ) {
    return (int)((r > 0.0) ? (r + 0.5) : (r - 0.5));
}
#endif

template<typename T> inline T sign(T x) {return (((x)>0) - ((x)<0));}

#define grl_assert(x) do { if (!(x)) { std::cerr << __FILE__ << ":" << __LINE__ << ": Assertion '" << #x << "' failed" << std::endl; abort(); } } while (0)

#endif /* GRL_COMPAT_H_ */
