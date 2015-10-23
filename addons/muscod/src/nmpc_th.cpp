/*
 * nmpc_th.h
 *
 *  Created on: Sep 12, 2015
 *      Author: Ivan Koryakovskiy
 */

#include <grl/policies/nmpc_th.h>
#include <dlfcn.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/stat.h>

using namespace grl;

REGISTER_CONFIGURABLE(NMPCPolicyTh)

// Initialize static variables
bool            NMPCPolicyTh::iv_ready_ = true;
pthread_t       NMPCPolicyTh::muscod_thread_;
pthread_cond_t  NMPCPolicyTh::cond_iv_ready_;
pthread_mutex_t NMPCPolicyTh::mutex_;
MUSCOD*         NMPCPolicyTh::muscod_ = NULL;

NMPCPolicyTh::~NMPCPolicyTh()
{
  muscod_quit(&data_);
  pthread_join(muscod_thread_, NULL);
  safe_delete(&muscod_);
}

void NMPCPolicyTh::request(ConfigurationRequest *config)
{
  config->push_back(CRP("model_name", "Name of MUSCOD model library", model_name_));
  config->push_back(CRP("lua_model", "Lua model used by MUSCOD", lua_model_));
  config->push_back(CRP("inputs", "int.observation_dims", "Number of inputs", (int)inputs_, CRP::System, 1));
  config->push_back(CRP("outputs", "int.action_dims", "Number of outputs", (int)outputs_, CRP::System, 1));
  config->push_back(CRP("single_step", "Run NMPC in single-step mode", (int)single_step_, CRP::System, 0, 1));
  config->push_back(CRP("verbose", "Verbose mode", (int)verbose_, CRP::System, 0, 1));
}

void NMPCPolicyTh::configure(Configuration &config)
{
  std::string model_path;
  model_path    = std::string(MUSCOD_CONFIG_DIR);
  model_name_   = config["model_name"].str();
  outputs_      = config["outputs"];
  inputs_       = config["inputs"];
  single_step_  = config["single_step"];
  verbose_      = config["verbose"];

  std::cout << "Running MUSCOD in a " << (single_step_?"single-step ":"multi-step ") << "mode." << std::endl;

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
  for (int ii=0; ii < 20; ++ii)  // TODO: check error instead
  {
    muscod_->nmpcFeedback(NULL, NULL, NULL);
    muscod_->nmpcTransition();
    muscod_->nmpcPrepare();
  }

  // save solver state
  data_.backup_muscod_state(muscod_);

  // prepare muscod structures
  muscod_obs_.resize(inputs_);

  // Copy initial solution to do warm control
  pthread_mutex_lock(&mutex_);
  muscod_action_ = data_.backup_qc;
  pthread_mutex_unlock(&mutex_);

  // Initialize mutex and condition variable objects
  pthread_mutex_init(&mutex_, NULL);
  pthread_cond_init (&cond_iv_ready_, NULL);

  // Start MUSCOD-II in a thread running a signal triggered execution loop
  pthread_mutex_lock(&mutex_);
  if (verbose_)
    std::cout << "In " << __func__ << ": creating MUSCOD-II thread" << std::endl;
  int rc = pthread_create(&muscod_thread_, NULL, muscod_run, static_cast<void*> (&data_));
  if (rc)
  {
      std::cerr << "ERROR: pthread_create() failed with " << rc << std::endl;
      std::cerr << "bailing out..." << rc << std::endl;
      abort();
  }

  // wait for MUSCOD thread to initialize data structure
  if (verbose_)
    std::cout << "MUSCOD: Waiting for MUSCOD thread..." << std::endl;

  pthread_cond_wait(&cond_iv_ready_, &mutex_);
  pthread_mutex_unlock(&mutex_);
}

void NMPCPolicyTh::muscod_reset(Vector &initial_obs, double time)
{
  // Wait for thread to stop at the condition variable
  pthread_mutex_lock(&mutex_);
  while (!iv_ready_) // TODO: do something with this!
  {
    pthread_mutex_unlock(&mutex_);
    usleep(100000);
    pthread_mutex_lock(&mutex_);
  }
  pthread_mutex_unlock(&mutex_);

  // Reload solution state
  data_.restore_muscod_state(muscod_);

  // Reinitialize state and time
  for (int IXD = 0; IXD < data_.NXD; ++IXD)
    data_.sd[IXD] = initial_obs[IXD];
  for (int IP = 0; IP < data_.NP; ++IP)
    data_.pf[IP] = time;

  // Solve until convergence to prepare solver
  for (int ii=0; ii < 20; ++ii) // TODO: check error instead
  {
    muscod_->nmpcFeedback(data_.sd.data(), data_.pf.data(), NULL);
    muscod_->nmpcTransition();
    muscod_->nmpcPrepare();
  }

  // Copy initial controls to provide immediate feedback
  for (int IMSN = 0; IMSN < data_.NMSN; ++IMSN)
    muscod_->getNodeQC (IMSN, data_.qc.row(IMSN).data());
  data_.qc = data_.backup_qc;

  // Reset counters
  qc_cnt_ = 0;
  qc_cnt_base_ = 0;

  if (verbose_)
    std::cout << "MUSCOD: reset done!" << std::endl;
}

void NMPCPolicyTh::reconfigure(const Configuration &config)
{
}

// MUSCOD-II main thread setup and execution loop
void *NMPCPolicyTh::muscod_run(void *indata)
{
  // cast data back to object
  if (indata == NULL) {
    std::cerr << "ERROR: indata is NULL" << std::endl;
    std::cerr << "bailing out..." << std::endl;
    abort();
  }
  MuscodData& data = *static_cast<MuscodData*> (indata);

  // instantiate values of data structure
  pthread_mutex_lock(&mutex_);
  unsigned long NMSN = data.NMSN;
  unsigned long NXD  = data.NXD;
  unsigned long NP   = data.NP;
  unsigned long NU   = data.NU;

  // define initial value and placeholder for feedback
  Vector sd = ConstantVector(NXD, 0.0);
  Vector pf = ConstantVector(NP,  0.0);
  Vector first_qc   = ConstantVector(NU,  0.0);

  // same for exchange TODO: move this to backup_muscod_state
  data.sd = ConstantVector(NXD, 0.0);
  data.pf = ConstantVector(NP,  0.0);
  data.qc = Matrix::Constant(NMSN, NU, 0.0);

  data.is_initialized = true;

  pthread_cond_signal(&cond_iv_ready_);
  pthread_mutex_unlock(&mutex_);

  std::cout << "MUSCOD: thread is ready!" << std::endl;

  // EXECUTION LOOP
  while (true)
  {
    pthread_mutex_lock(&mutex_);
    if (data.quit)
    {
      pthread_mutex_unlock(&mutex_);
      break; // exit after finishing calculations
    }

    // PUT CONTROLS INTO DATA STRUCTURE
    for (int IMSN = 0; IMSN < NMSN; ++IMSN)
      muscod_->getNodeQC (IMSN, data.qc.row(IMSN).data());
    iv_ready_ = true; // Let know main thread that controls are ready and thread requires new state

    pthread_cond_wait(&cond_iv_ready_, &mutex_);
    if (data.quit)
    {
      pthread_mutex_unlock(&mutex_);
      break; // exit when thread is blocked
    }

    {
      // GET INITIAL VALUES FROM DATA
      for (int IXD = 0; IXD < NXD; ++IXD)
        sd[IXD] = data.sd[IXD];

      // GET GLOBAL PARAMETERS FROM DATA
      for (int IP = 0; IP < NP; ++IP)
        pf[IP] = data.pf[IP];
    }
    pthread_mutex_unlock(&mutex_);

    // NMPC loop, assume that it converges after 3 iterations
    for (int ii=0; ii < 4; ++ii)
    {
      muscod_->nmpcFeedback(sd.data(), pf.data(), first_qc.data());
      muscod_->nmpcTransition();
//      muscod_->nmpcShift(3); TODO: Second iteration does not work
      muscod_->nmpcPrepare();
    }
  } // END OF EXECUTION LOOP

  std::cout << "MUSCOD: Exit thread" << std::endl;
  pthread_exit(NULL);
}

// MUSCOD-II main thread cleanup
void NMPCPolicyTh::muscod_quit(void* data)
{
  //release the thread from waiting if needed
  pthread_mutex_lock(&mutex_);
  static_cast<MuscodData*> (data)->quit = true;
  pthread_cond_signal(&cond_iv_ready_);
  pthread_mutex_unlock(&mutex_);
}

void NMPCPolicyTh::act(double time, const Vector &in, Vector *out)
{
  if (time == 0.0)
  {
    so_convert_obs_for_muscod(NULL, NULL);                    // Reset internal counters
    so_convert_obs_for_muscod(in.data(), muscod_obs_.data()); // Convert
    muscod_reset(muscod_obs_, time);
  }

  out->resize(outputs_);

  if (verbose_)
    std::cout << "observation state: [ " << in << "]" << std::endl;

  // Convert MPRL states into MUSCOD states
  so_convert_obs_for_muscod(in.data(), muscod_obs_.data());

  if (verbose_)
    std::cout << "time: [ " << time << " ]; state: [ " << muscod_obs_ << "]" << std::endl;

  // Obtain feedback, provide new state and unblock thread
  pthread_mutex_lock(&mutex_);
  if (iv_ready_)
  {
    // Obtain feedback
    muscod_action_ = data_.qc;

    if (verbose_)
      std::cout << "Obtained Control - 1:"
                << muscod_action_.block(0,0,10,data_.NU).transpose() << std::endl;

    // Provide state and time
    for (int IXD = 0; IXD < data_.NXD; ++IXD)
      data_.sd[IXD] = muscod_obs_[IXD];
    for (int IP = 0; IP < data_.NP; ++IP)
      data_.pf[IP] = time;

    iv_ready_ = false;
    // signal to MUSCOD thread to embed initial values and start the computation
    pthread_cond_signal(&cond_iv_ready_);

    if (single_step_)
    {
      while (!iv_ready_) // wait for thread to finish computations
      {
        pthread_mutex_unlock(&mutex_);
        usleep(100000);
        pthread_mutex_lock(&mutex_);
      }
      // in single-step mode use feedback immediately
      qc_cnt_base_ = 0;
      muscod_action_ = data_.qc;
    }

    if (verbose_)
    {
      std::cout << "Obtained Controls:"
                << muscod_action_.block(0,0,10,data_.NU).transpose() << std::endl;
      std::cout << "Start from " << qc_cnt_base_ << std::endl;
    }

    qc_cnt_ = qc_cnt_base_;
    qc_cnt_base_ = 0;
  }
  pthread_mutex_unlock(&mutex_);

  // Retrieve values for policy during MUSCOD calculations
  if (qc_cnt_ < muscod_action_.size())
  {
    if (verbose_)
      std::cout << "Feedback Control:" << muscod_action_.row(qc_cnt_) << std::endl;
    *out = muscod_action_.row(qc_cnt_);
    qc_cnt_++;
    qc_cnt_base_++;
  }
  else
  {
    std::cerr << "ERROR: MUSCOD-II was too slow." << std::endl;
    std::cerr << "       No more controls for given horizon!" << std::endl;
    std::cerr << "Abort and cleanup..." << std::endl;
  }
}

NMPCPolicyTh *NMPCPolicyTh::clone() const
{
  NMPCPolicyTh* obj = new NMPCPolicyTh(*this);
  obj->reset();
  return obj;
}

void NMPCPolicyTh::print_array(const double* arr, const unsigned int len)
{
  std::cout << " [";
  for (int i = 0; i < len; ++i) {
    std::cout << arr[i];
    if (i < len-1) {
      std::cout << ", ";
    }
  }
  std::cout << "]  ";
}



