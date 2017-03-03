/** \file importer.h
 * \brief Variable importer header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-09-25
 *
 * \copyright \verbatim
 * Copyright (c) 2015, Wouter Caarls and Ivan Koryakovskiy
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

#ifndef GRL_IMPORTER_H_
#define GRL_IMPORTER_H_

#include <grl/configurable.h>

namespace grl {

class Importer : public Configurable
{
  public:
    /// Register header names of variables that will be read.
    virtual void init(const std::vector<std::string> &headers) = 0;

    /// Open a file.
    virtual void open(const std::string &variant="") = 0;

    /**
     * \brief Read a line.
     * 
     * The variable list should correspond to the header name.
     */
    virtual bool read(const std::vector<Vector*> &vars) = 0;

    /**
     * \brief Read a line into a single variable.
     * 
     * Reads all specified headers.
     */
    virtual bool read(Vector *var) = 0;
};

class CSVImporter : public Importer
{
  public:
    TYPEINFO("importer/csv", "Comma-separated values importer");
    
  protected:
    std::string file_;
    std::string fields_;
    
    std::ifstream stream_;
    std::vector<int> order_;
    std::vector<std::string> headers_;

  public:
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Importer
    virtual void init(const std::vector<std::string> &headers);
    virtual void open(const std::string &variant="");
    virtual bool read(const std::vector<Vector*> &vars);
    virtual bool read(Vector *var);
};

}

#endif // GRL_IMPORTER_H_
