// *****************************************************************************
// Includes
// *****************************************************************************

#include <iostream>
#include <dlfcn.h>
#include <sys/stat.h>

// GRL
#include <grl/policies/nmpc.h>

// MUSCOD-II interface
#include <wrapper.hpp>

using namespace grl;

REGISTER_CONFIGURABLE(NMPCPolicy)

NMPCPolicy::~NMPCPolicy()
{
  safe_delete(&muscod_nmpc_);
}

void NMPCPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("lua_model", "Lua model used by MUSCOD", lua_model_));
  config->push_back(CRP("nmpc_model_name", "Name of MUSCOD MHE model library", nmpc_model_name_));
  config->push_back(CRP("outputs", "int.action_dims", "Number of outputs", (int)outputs_, CRP::System, 1));
  config->push_back(CRP("verbose", "Verbose mode", (int)verbose_, CRP::System, 0, 1));
}

void *NMPCPolicy::setup_model_path(const std::string path, const std::string model, const std::string lua_model)
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

void NMPCPolicy::configure(Configuration &config)
{
  std::string model_path;
  model_path        = std::string(MUSCOD_CONFIG_DIR);
  nmpc_model_name_  = config["nmpc_model_name"].str();
  outputs_          = config["outputs"];
  verbose_          = config["verbose"];

  // Setup path for the problem description library and lua, csv, dat files used by it
  std::string problem_path  = model_path + "/" + nmpc_model_name_;

  //-------------------- Load Lua model which is used by muscod ------------------- //
  lua_model_ = problem_path + "/" + config["lua_model"].str();

  struct stat buffer;
  if (stat(lua_model_.c_str(), &buffer) != 0) // check if lua file exists in the problem description folder
    lua_model_ = std::string(RBDL_LUA_CONFIG_DIR) + "/" + config["lua_model"].str(); // if not, then use it as a reference from dynamics

  //----------------- Set path in the problem description library ----------------- //
  void * so_handle_nmpc = setup_model_path(problem_path, nmpc_model_name_, lua_model_);

  //----------------- Observation converter ----------------- //
/*  so_convert_obs_for_muscod_ = (t_obs_converter) dlsym(so_handle_nmpc, "convert_obs_for_muscod");
  if (so_convert_obs_for_muscod_ == NULL)
  {
    std::cout << "ERROR: Could not symbol in shared library: 'convert_obs_for_muscod'" << std::endl;
    std::cout << "bailing out ..." << std::endl;
    std::exit(-1);
  }
*/
  //------------------- Initialize NMPC ------------------- //
  muscod_nmpc_ = new MUSCOD();
  nmpc_ = new NMPCProblem(problem_path.c_str(), nmpc_model_name_.c_str(), muscod_nmpc_);

  // Allocate memory
  initial_sd_ = VectorConstructorFill(nmpc_->NXD(), 0);
  initial_pf_ = VectorConstructorFill(nmpc_->NP(), 0);
  initial_qc_ = VectorConstructorFill(nmpc_->NU(), 0);
  final_sd_   = VectorConstructorFill(nmpc_->NXD(), 0);

  // FIXME This part is needed
  // if (!verbose_) {
    // muscod_mhe_->setLogLevelScreen(-1);
    // muscod_mhe_->setLogLevelAndFile(-1, NULL, NULL);
    // muscod_nmpc_->setLogLevelScreen(-1);
    // muscod_nmpc_->setLogLevelAndFile(-1, NULL, NULL);
  // }

/*
  // save solver state
  // FIXME this part is needed
  data_.backup_muscod_state(muscod_);
  data_.sd = ConstantVector(data_.NXD, 0.0);
  data_.pf = ConstantVector(data_.NP,  0.0);
*/

  if (verbose_)
    std::cout << "MUSCOD is ready!" << std::endl;
}

void NMPCPolicy::reconfigure(const Configuration &config)
{
}


void NMPCPolicy::muscod_reset(const Vector &initial_obs, double time)
{
  // FIXME
  // load solution state
  // data_.restore_muscod_state(muscod_);

  // // Reinitialize state and time
  // for (int IP = 0; IP < data_.NP; ++IP)
  //   data_.pf[IP] = time;

  // initialize NMPC
  for (int inmpc = 0; inmpc < 10; ++inmpc) {
    // 1) Feedback: Embed parameters and initial value from MHE
    nmpc_->feedback();
    // 2) Transition
    nmpc_->transition();
    // 3) Preparation
    nmpc_->preparation();
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
//  if (verbose_)
//    std::cout << "observation state: [ " << in << "]" << std::endl;

//  Vector obs;
//  obs.resize(in.size());
  if (time == 0.0)
  {
//    so_convert_obs_for_muscod_(NULL, NULL);            // Reset internal counters
//    so_convert_obs_for_muscod_(in.data(), obs.data()); // Convert
    muscod_reset(in, time);
  }
  // Convert MPRL states into MUSCOD states
//  so_convert_obs_for_muscod_(in.data(), obs.data());

  if (verbose_)
    std::cout << "time: [ " << time << " ]; state: [ " << in << "]" << std::endl;

  out->resize(outputs_);
  //  for (int IP = 0; IP < data_.NP; ++IP)
  //    data_.pf[IP] = time;

  if (time <= 0.0) {
    initial_sd_ << in;
    initial_pf_ << 0.0;
    initial_qc_ << 0.0;
  } else {
    std::cout << "time: [ " << time << " ]" << std::endl;
    std::cout << "observer  = " << in << std::endl;
    std::cout << "final_sd_ = " << final_sd_ << std::endl;
    std::cout << "ERROR     = " << final_sd_ - in << std::endl;
  }

  // Run multiple NMPC iterations
  const unsigned int nnmpc = 10;
  for (int inmpc = 0; inmpc < nnmpc; ++inmpc) {
    // 1) Feedback: Embed parameters and initial value from MHE
    // NOTE the same initial values (sd, pf) are embedded several time,
    //      but this will result in the same solution as running a MUSCOD
    //      instance for several iterations
    nmpc_->feedback(in, initial_pf_, &initial_qc_);
    // 2) Transition
    nmpc_->transition();
    // 3) Shifting
    // NOTE do that only once at last iteration
    if (nnmpc > 0 && inmpc == nnmpc-1) {
      nmpc_->shifting(1);
    }
    // 4) Preparation
    nmpc_->preparation();
  }

  Vector real_pf;
  real_pf = VectorConstructorFill(nmpc_->NP(), 0);

  // Simulate
  nmpc_->simulate(
      in,
      real_pf,
      initial_qc_,
      0.05,
      &final_sd_
  );

  // Here we can return the feedback control
  (*out) = initial_qc_;

  if (verbose_)
    std::cout << "Feedback Control: [" << *out << "]" << std::endl;
}

