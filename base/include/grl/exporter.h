/** \file exporter.h
 * \brief Variable exporter header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2015-07-15
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

#ifndef GRL_EXPORTER_H_
#define GRL_EXPORTER_H_

#include <grl/configurable.h>

namespace grl {

class Exporter : public Configurable
{
  public:
    /// Register header names of variables that will be written.
    virtual void init(const std::vector<std::string> &headers) = 0;
    
    /// Open a file.
    virtual void open(const std::string &variant="", bool append=true) = 0;

    /**
     * \brief Write a line.
     * 
     * The variable list should correspond to the header name.
     */
    virtual void write(const std::vector<Vector> &vars) = 0;

    /**
     * \brief Append to a line.
     *
     * Allows to write a line in multiple calls.
     * The variable list should contain all or a few of header names.
     * After the line is fully prepared it is written autamatically.
     */
    virtual void append(const std::vector<Vector> &vars) = 0;
};

class CSVExporter : public Exporter
{
  public:
    TYPEINFO("exporter/csv", "Comma-separated values exporter");
    
  protected:
    std::string file_;
    std::string fields_;
    std::string style_;
    std::string variant_;
    
    std::ofstream stream_;
    std::vector<size_t> order_;
    std::vector<std::string> headers_;
    std::vector<Vector> append_vec_;
    bool write_header_, enabled_;
    std::map<std::string, int> run_counter_;

  public:
    CSVExporter() : style_("line"), variant_("all"), write_header_(true), enabled_(true) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Exporter
    void init(const std::vector<std::string> &headers);
    void open(const std::string &variant="", bool append=true);
    void write(const std::vector<Vector> &vars);
    void append(const std::vector<Vector> &vars);
};

}

#endif // GRL_EXPORTER_H_
