/** \file nmpc_base.h
 * \brief Base class for NMPC.
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

#ifndef GRL_NMPC_BASE_POLICY_H_
#define GRL_NMPC_BASE_POLICY_H_

#include <grl/policy.h>
#include <grl/policies/muscod_nmpc.h>

class MUSCOD;

namespace grl
{

/// NMPCBase policy
class NMPCBase : public Policy
{
  protected:
    int verbose_;
    int initFeedback_;
    std::string model_name_, lua_model_, nmpc_model_name_, model_path_;
    size_t outputs_, inputs_;
    Vector action_min_, action_max_;

  public:
    NMPCBase() : initFeedback_(0), inputs_(0), outputs_(0), verbose_(0) {}
    //virtual ~NMPCBase() { }

    // From Configurable
    virtual void request(ConfigurationRequest *config)
    {
      config->push_back(CRP("action_min", "vector.action_min", "Lower limit on actions", action_min_, CRP::System));
      config->push_back(CRP("action_max", "vector.action_max", "Upper limit on actions", action_max_, CRP::System));
      config->push_back(CRP("lua_model", "Lua model used by MUSCOD", lua_model_));
      config->push_back(CRP("model_name", "Name of the model in grl", model_name_));
      config->push_back(CRP("nmpc_model_name", "Name of MUSCOD MHE model library", nmpc_model_name_));
      config->push_back(CRP("outputs", "int.action_dims", "Number of outputs", (int)outputs_, CRP::System, 1));
      config->push_back(CRP("initFeedback", "Initialize feedback", (int)initFeedback_, CRP::System, 0, 1));
      config->push_back(CRP("verbose", "Verbose mode", (int)verbose_, CRP::System, 0, 1));
    }

    virtual void configure(Configuration &config)
    {
      model_path_       = std::string(MUSCOD_CONFIG_DIR);
      nmpc_model_name_  = config["nmpc_model_name"].str();
      model_name_       = config["model_name"].str();
      outputs_          = config["outputs"];
      verbose_          = config["verbose"];
      action_min_       = config["action_min"].v();
      action_max_       = config["action_max"].v();

      if (action_min_.size() != action_max_.size())
        throw bad_param("policy/nmpc:{action_min, action_max}");
    }

  protected:
    void *setup_model_path(const std::string path, const std::string model, const std::string lua_model)
    {
      // get the library handle,
      std::string so_path  = path + "/" + "lib" + model + ".so";
      void *so_handle = dlopen(so_path.c_str(), RTLD_NOW|RTLD_GLOBAL);
      if (so_handle==NULL)
      {
        std::cout << "ERROR: Could not load MUSCOD-II shared model library: '" << so_path << "'" << std::endl;
        std::cout << "dlerror responce: " << dlerror() << std::endl;
        std::cout << "bailing out ..." << std::endl;
        exit(EXIT_FAILURE);
      }

      // get the function handle
      void (*so_set_path)(std::string, std::string);
      std::string so_set_path_fn = "set_path"; // name of a function which sets the path
      so_set_path = (void (*)(std::string, std::string)) dlsym(so_handle, so_set_path_fn.c_str());
      if (so_set_path==NULL)
      {
        std::cout << "ERROR: Could not symbol in shared library: '" << so_set_path_fn << "'" << std::endl;
        std::cout << "bailing out ..." << std::endl;
        std::exit(-1);
      }

      // ... and finally set the paths
      if (verbose_)
      {
        std::cout << "MUSCOD: setting new problem path to: '" << path << "'" <<std::endl;
        std::cout << "MUSCOD: setting new Lua model file to: '" << lua_model << "'" <<std::endl;
      }
      so_set_path(path, lua_model);

      return so_handle;
    }
};

}

#endif /* GRL_NMPC_BASE_POLICY_H_ */
