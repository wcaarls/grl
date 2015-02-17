/** \file memstring.h
 * \brief Encapsulation of Matlab strings.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-17
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
 
#ifndef GRL_MATLAB_MEMSTRING_H_
#define GRL_MATLAB_MEMSTRING_H_

#include <string>
#include <mex.h>

class MexMemString
{
  private:
    char *string_;

  public:
    MexMemString() : string_(NULL) { }
    ~MexMemString()
    {
      if (string_)
        mxFree(string_);
    }

    operator char*()
    {
      return string_;
    }

    operator std::string()
    {
      return std::string(string_);
    }

    MexMemString &operator=(char *rhs)
    {
      if (string_)
        mxFree(string_);

      string_ = rhs;

      return *this;
    }
};

#endif /* GRL_MATLAB_MEMSTRING_H_ */
