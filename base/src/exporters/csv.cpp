/** \file exporter.h
 * \brief Comma separated values exporter source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
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

#include <iomanip>
#include <sys/stat.h>
#include <grl/exporter.h>

using namespace grl;

REGISTER_CONFIGURABLE(CSVExporter)

void CSVExporter::request(ConfigurationRequest *config)
{
  config->push_back(CRP("file", "Output base filename", file_));
  config->push_back(CRP("fields", "Comma-separated list of fields to write", fields_));
  config->push_back(CRP("style", "Header style", style_, CRP::Configuration, {"none", "line", "meshup"}));
  config->push_back(CRP("variant", "Variant to export", variant_, CRP::Configuration, {"test", "learn", "all"}));
}

void CSVExporter::configure(Configuration &config)
{
  file_    = config["file"].str();
  fields_  = config["fields"].str();
  style_   = config["style"].str();
  variant_ = config["variant"].str();
  
  if (file_.empty())
    throw bad_param("exporter/csv:file");
}

void CSVExporter::reconfigure(const Configuration &config)
{
}

void CSVExporter::init(const std::initializer_list<std::string> &headers)
{
  order_.clear();
  headers_ = headers;
  
  if (fields_.empty())
  {
    // No fields requested. Write all.
    for (size_t ii=0; ii < headers.size(); ++ii)
      order_.push_back(ii);
  }
  else
  {
    std::string fields = fields_;

    // Iterate over requested fields
    do
    {
      size_t sep;
      std::string field;
      
      if ((sep = fields.find(',')) != std::string::npos)
      {
        field = fields.substr(0, sep);
        fields = fields.substr(sep+1);
      }
      else
      {
        // Last field
        field = fields;
        fields = "";
      }
      
      // Trim
      size_t ws;
      while ((ws = field.find_first_of(" \t")) != std::string::npos)
        field.erase(ws, 1);

      // Find associated header
      bool found=false;
      for (size_t ii=0; ii < headers_.size(); ++ii)
        if (headers_[ii] == field)
        {
          order_.push_back(ii);
          found = true;
        }
        
      if (!found)
      {
        ERROR("Requested unregistered field '" << field << "'");
        throw bad_param("exporter/csv:fields");
      }
    } while (fields != "");
  }
}

void CSVExporter::open(const std::string &variant, bool append)
{
  if (stream_.is_open())
    stream_.close();

  // Check if our export objective matches exporter variant
  if (variant_ != "all" && variant_ != variant)
    return;

  // Append variant if given
  std::string file = file_;
  if (!variant.empty())
    file = file_ + "-" + variant;

  // Append run
  if (!append)
    run_counter_[file]++;
  file = file + "-" + std::to_string(run_counter_[file] - 1);

  // Write header when opening a new or empty file
  struct stat buffer;
  if (stat((file+".csv").c_str(), &buffer) != 0 || !buffer.st_size || !append)
    write_header_ = true;
  else
    write_header_ = false;

  stream_.open((file+".csv").c_str(), std::ofstream::out | (append?std::ofstream::app:std::ofstream::trunc));
}

void CSVExporter::write(std::vector<Vector> vars)
{
  if (!stream_.is_open())
    return;

  if (vars.size() != headers_.size())
  {
    ERROR("Variable list does not match header list");
    return;
  }

  if (write_header_ && style_ != "none")
  {
    if (style_ == "meshup")
      stream_ << "COLUMNS:" << std::endl;

    for (size_t ii=0; ii < order_.size(); ++ii)
    {
      Vector &v = vars[order_[ii]];
      for (size_t jj = 0; jj < v.size(); ++jj)
      {
        stream_ << headers_[order_[ii]] << "[" << jj << "]";
        if (ii < order_.size()-1 || jj < v.size() -1)
          stream_ << ", ";
        if (style_ == "meshup")
          stream_ << std::endl;
      }
    }

    if (style_ == "meshup")
      stream_ << "DATA:" << std::endl;
    else
      stream_ << std::endl;
  }

  write_header_ = false;

  for (size_t ii=0; ii < order_.size(); ++ii)
  {
    Vector &v = vars[order_[ii]];
    for (size_t jj = 0; jj < v.size(); ++jj)
    {
      stream_ << std::fixed << std::setw(11) << std::setprecision(6) << v[jj];
      if (ii < order_.size()-1 || jj < v.size() -1)
        stream_ << ", ";
    }
  }

  stream_ << std::endl;
}

void CSVExporter::write(const std::initializer_list<Vector> &vars)
{
  std::vector<Vector> var_vec;
  var_vec.insert(var_vec.end(), vars.begin(), vars.end());
  write(var_vec);
}

void CSVExporter::append(const std::initializer_list<Vector> &vars)
{
  append_vec_.insert(append_vec_.end(), vars.begin(), vars.end());
  if (append_vec_.size() >= headers_.size())
  {
    write(append_vec_);
    append_vec_.clear();
  }
}
