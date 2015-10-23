/** \file nmpc.cpp
 * \brief NMPC policy source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-05-08
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

#include <grl/policies/nmpc.h>
#include <wrapper.hpp>
#include <dlfcn.h>
#include <sys/stat.h>

using namespace grl;

REGISTER_CONFIGURABLE(NMPCPolicy)

NMPCPolicy::~NMPCPolicy()
{
  safe_delete(&muscod_);
}

void NMPCPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("lua_model", "Lua model used by MUSCOD", lua_model_));
  config->push_back(CRP("model_name", "Name of MUSCOD model library", model_name_));
  config->push_back(CRP("outputs", "int.action_dims", "Number of outputs", (int)outputs_, CRP::System, 1));
  config->push_back(CRP("verbose", "Verbose mode", (int)verbose_, CRP::System, 0, 1));
}

void NMPCPolicy::configure(Configuration &config)
{
  std::string model_path;
  model_path    = std::string(MUSCOD_CONFIG_DIR);
  model_name_   = config["model_name"].str();
  outputs_      = config["outputs"];
  verbose_      = config["verbose"];

  // Setup path for the problem description library and lua, csv, dat files used by it
  std::string problem_path  = model_path + "/" + model_name_;

  //-------------------- Load Lua model which is used by muscod ------------------- //
  lua_model_ = problem_path + "/" + config["lua_model"].str();

  struct stat buffer;
  if (stat(lua_model_.c_str(), &buffer) != 0) // check if lua file exists in the problem description folder
    lua_model_ = std::string(RBDL_LUA_CONFIG_DIR) + "/" + config["lua_model"].str(); // if not, then use it as a reference from dynamics

  //----------------- Set path in the problem description library ----------------- //
  // get the library handle,
  std::string so_path  = problem_path + "/" + "lib" + model_name_ + ".so";
  so_handle_ = dlopen(so_path.c_str(), RTLD_NOW|RTLD_GLOBAL);
  if (so_handle_==NULL)
  {
    std::cout << "ERROR: Could not load MUSCOD-II shared model library: '" << so_path << "'" << std::endl;
    std::cout << "dlerror responce: " << dlerror() << std::endl;
    std::cout << "bailing out ..." << std::endl;
    exit(EXIT_FAILURE);
  }

  // get the function handle
  void (*so_set_path)(std::string, std::string);
  std::string so_set_path_fn = "set_path"; // name of a function which sets the path
  so_set_path = (void (*)(std::string, std::string)) dlsym(so_handle_, so_set_path_fn.c_str());
  if (so_set_path==NULL)
  {
    std::cout << "ERROR: Could not symbol in shared library: '" << so_set_path_fn << "'" << std::endl;
    std::cout << "bailing out ..." << std::endl;
    std::exit(-1);
  }

  // ... and finally set the paths
  if (verbose_)
  {
    std::cout << "MUSCOD: setting new problem path to: '" << problem_path << "'" <<std::endl;
    std::cout << "MUSCOD: setting new Lua model file to: '" << lua_model_ << "'" <<std::endl;
  }
  so_set_path(problem_path, lua_model_);

  //----------------- Observation converter ----------------- //
  so_convert_obs_for_muscod = (void (*)(const double *from, double *to)) dlsym(so_handle_, "convert_obs_for_muscod");
  if (so_convert_obs_for_muscod==NULL)
  {
    std::cout << "ERROR: Could not symbol in shared library: 'convert_obs_for_muscod'" << std::endl;
    std::cout << "bailing out ..." << std::endl;
    std::exit(-1);
  }

  //------------------- Initialize MUSCOD ------------------- //
  muscod_ = new MUSCOD();
  muscod_->setModelPathAndName(problem_path.c_str(), model_name_.c_str());
  muscod_->loadFromDatFile(".", model_name_.c_str());
  muscod_->setLogLevelScreen(-1);
  muscod_->setLogLevelAndFile(-1, NULL, NULL);

  // get proper dimensions
  muscod_->nmpcInitialize(muscod_->getSSpec(), NULL, NULL);

  // solve until convergence to prepare solver
  for (int ii=0; ii < 20; ++ii)
  {
    muscod_->nmpcFeedback(NULL, NULL, NULL);
    muscod_->nmpcTransition();
    muscod_->nmpcPrepare();
  }

  // save solver state
  data_.backup_muscod_state(muscod_);
  data_.sd = ConstantVector(data_.NXD, 0.0);
  data_.pf = ConstantVector(data_.NP,  0.0);

  if (verbose_)
    std::cout << "MUSCOD is ready!" << std::endl;
}

void NMPCPolicy::reconfigure(const Configuration &config)
{
}


void NMPCPolicy::muscod_reset(Vector &initial_obs, double time)
{
  // load solution state
  data_.restore_muscod_state(muscod_);

  // Reinitialize state and time
  for (int IP = 0; IP < data_.NP; ++IP)
    data_.pf[IP] = time;

  // solve until convergence to prepare solver
  for (int ii=0; ii < 20; ++ii)
  {
    muscod_->nmpcFeedback(initial_obs.data(), data_.pf.data(), NULL);
    muscod_->nmpcTransition();
    muscod_->nmpcPrepare();
  }

  if (verbose_)
    std::cout << "MUSCOD is reseted!" << std::endl;
}

NMPCPolicy *NMPCPolicy::clone() const
{
  return NULL;
}

void NMPCPolicy::act(double time, const Vector &in, Vector *out)
{
  if (verbose_)
    std::cout << "observation state: [ " << in << "]" << std::endl;

  Vector obs;
  obs.resize(in.size());
  if (time == 0.0)
  {
    so_convert_obs_for_muscod(NULL, NULL);            // Reset internal counters
    so_convert_obs_for_muscod(in.data(), obs.data()); // Convert
    muscod_reset(obs, time);
  }

  // Convert MPRL states into MUSCOD states
  so_convert_obs_for_muscod(in.data(), obs.data());

  if (verbose_)
    std::cout << "time: [ " << time << " ]; state: [ " << obs << "]" << std::endl;

  out->resize(outputs_);
  for (int IP = 0; IP < data_.NP; ++IP)
    data_.pf[IP] = time;
  for (int ii=0; ii < 10; ++ii)
  {
    muscod_->nmpcFeedback(obs.data(),  data_.pf.data(), out->data());
    muscod_->nmpcTransition();
//    muscod_->nmpcShift(3); TODO: Doesn't work for cart-pole
    muscod_->nmpcPrepare();
  }

  if (verbose_)
    std::cout << "Feedback Control: [" << *out << "]" << std::endl;
}
