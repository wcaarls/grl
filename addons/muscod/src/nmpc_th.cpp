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
  so_convert_obs_for_muscod = (void (*)(const std::vector<double>&, std::vector<double>&))
          dlsym(so_handle_, "convert_obs_for_muscod");
  if (so_convert_obs_for_muscod==NULL)
  {
    std::cout << "ERROR: Could not symbol in shared library: 'convert_obs_for_muscod'" << std::endl;
    std::cout << "bailing out ..." << std::endl;
    std::exit(-1);
  }

  //------------------- Initialize MUSCOD ------------------- //
//  data_.model_path = problem_path;
//  data_.relative_dat_path = ".";
//  data_.model_name = model_name_;

  muscod_ = new MUSCOD();
  muscod_->setModelPathAndName(problem_path.c_str(), model_name_.c_str());
  muscod_->loadFromDatFile(".", model_name_.c_str());

//  muscod_->setModelPathAndName(model_path_.c_str(), model_name_.c_str());
//  muscod_->loadFromDatFile(NULL, NULL);
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

  NMSN_ = data_.NMSN;
  NXD_  = data_.NXD;
  NP_   = data_.NP;
  NU_   = data_.NU;

  // prepare muscod structures
  muscod_obs_.resize(inputs_);
  muscod_action_.resize(NMSN_);
  for (int i = 0; i < NMSN_; i++)
    muscod_action_[i].resize(outputs_);

  // Copy initial solution to do warm control
  pthread_mutex_lock(&mutex_);
  for (int IMSN = 0; IMSN < NMSN_; ++IMSN)
    muscod_action_[IMSN] = data_.backup_qc[IMSN];
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

  //muscod_init();
}

void NMPCPolicyTh::muscod_init()
{/*
  // Prepare to start thread
  qc_cnt_ = 0;
  qc_cnt_base_ = 0;

  // Be sure thread will not quit (since thread has not started yet it is safe)
  data_.quit = false;

  // Initialize mutex and condition variable objects
  pthread_mutex_init(&mutex_, NULL);
  pthread_cond_init (&cond_iv_ready_, NULL);

  // Start MUSCOD-II in a thread running a signal triggered execution loop
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
  pthread_mutex_lock(&mutex_);
  pthread_cond_wait(&cond_iv_ready_, &mutex_);
  pthread_mutex_unlock(&mutex_);

  // get dimensions from data structure
  unsigned long NMSN, NXD, NXA, NU, NP;
  pthread_mutex_lock(&mutex_);
  NMSN_ = data_.NMSN;
  NXD_  = data_.NXD;
  NXA_  = data_.NXA;
  NU_   = data_.NU;
  NP_   = data_.NP;
  pthread_mutex_unlock(&mutex_);

  muscod_obs_.resize(inputs_);
  muscod_action_.resize(NMSN_);
  for (int i = 0; i < NMSN_; i++)
    muscod_action_[i].resize(outputs_);

  // Copy initial solution to do warm control
  pthread_mutex_lock(&mutex_);
  for (int IMSN = 0; IMSN < NMSN_; ++IMSN)
    muscod_action_[IMSN] = data_.qc[IMSN];
  pthread_mutex_unlock(&mutex_);
  */
}

void NMPCPolicyTh::muscod_reset(Vector &initial_obs, double time)
{
  // Finalise thread from previous episode (if required)
//  muscod_quit(&data_);
//  int r = pthread_join(muscod_thread_, NULL);
//  if (r)
//    std::cout << "pthread_join resulted in error: " << strerror(r) << std::endl;

  // Wait for thread to stop at the condition variable
  pthread_mutex_lock(&mutex_);
  while (!iv_ready_)
  {
    pthread_mutex_unlock(&mutex_);
    usleep(100000);
    pthread_mutex_lock(&mutex_);
  }
  pthread_mutex_unlock(&mutex_);

  // Reload solution state
  data_.restore_muscod_state(muscod_);

  // Reinitialize state and time
  for (int IXD = 0; IXD < NXD_; ++IXD)
    data_.initial_sd[IXD] = initial_obs[IXD];
  for (int IP = 0; IP < NP_; ++IP)
    data_.initial_pf[IP] = time;

  // Solve until convergence to prepare solver
  for (int ii=0; ii < 100; ++ii) // TODO: check error instead
  {
    muscod_->nmpcFeedback(data_.initial_sd.data(), data_.initial_pf.data(), NULL);
    muscod_->nmpcTransition();
    muscod_->nmpcPrepare();
  }

  // Copy initial controls to provide immediate feedback
  for (int IMSN = 0; IMSN < NMSN_; ++IMSN)
    muscod_->getNodeQC (IMSN, &data_.qc[IMSN][0]);
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

/*
  // NMPC SETUP
  muscod_->setLogLevelScreen(-1);
  muscod_->setLogLevelAndFile(-1, NULL, NULL);
  muscod_->setModelPathAndName(data.model_path.c_str(), data.model_name.c_str());
  muscod_->loadFromDatFile(data.relative_dat_path.c_str(), data.model_name.c_str());
  muscod_->sqpInitialize(muscod_->getSSpec(), NULL, NULL);
  muscod_->sqpSolve();
  muscod_->options->wflag = MS_WARM;

  // get dimensions from MUSCOD instance
  unsigned long NMSN = muscod_->getNMSN(0);
  unsigned long NXD, NXA, NU, NP;
  muscod_->getDimIMSN(0, &NXD, &NXA, &NU);
  NP = muscod_->data.dataMSSQP->vdim.pf;
*/

  // instantiate values of data structure
  pthread_mutex_lock(&mutex_);
/*  data.NMSN = NMSN;
  data.NXD  = NXD;
  data.NXA  = NXA;
  data.NU   = NU;
  data.NP   = NP;
*/
  unsigned long NMSN = data.NMSN;
  unsigned long NXD  = data.NXD;
  unsigned long NP   = data.NP;
  unsigned long NU   = data.NU;

  // define initial value and placeholder for feedback
  std::vector<double> initial_sd (NXD, 0.0);
  std::vector<double> initial_pf (NP,  0.0);
  std::vector<double> first_qc   (NU,  0.0);

  // same for exchange TODO: move this to backup_muscod_state
  data.initial_sd = std::vector<double> (NXD, 0.0);
  data.initial_pf = std::vector<double> (NP,  0.0);
  data.qc = std::vector<std::vector<double>> (NMSN, first_qc);

  data.is_initialized = true;
  // PUT CONTROLS INTO DATA STRUCTURE: should be avaliable to a user right away
/*  for (int IMSN = 0; IMSN < NMSN; ++IMSN) {
    muscod_->getNodeQC (IMSN, &data.qc[IMSN][0]);
  }
*/
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
      muscod_->getNodeQC (IMSN, &data.qc[IMSN][0]);
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
        initial_sd[IXD] = data.initial_sd[IXD];

      // GET GLOBAL PARAMETERS FROM DATA
      for (int IP = 0; IP < NP; ++IP)
        initial_pf[IP] = data.initial_pf[IP];
    }
    pthread_mutex_unlock(&mutex_);

    // NMPC loop, assume that it converges after 3 iterations
    for (int ii=0; ii < 4; ++ii)
    {
      muscod_->nmpcFeedback(initial_sd.data(), initial_pf.data(), first_qc.data());
      muscod_->nmpcTransition();
//      muscod_->nmpcShift(3);
      muscod_->nmpcPrepare();
    }


    // EMBED INITIAL VALUES AND PARAMETERS
//    muscod_->nmpcEmbedding (&initial_sd[0], &initial_pf[0]);

    // SOLVE UNTIL CONVERGENCE
//    muscod_->sqpSolve();
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
    // Reset internal counters
    std::vector<double> dummy;
    so_convert_obs_for_muscod(dummy, dummy);
    // Convert
    so_convert_obs_for_muscod(in, muscod_obs_);
    muscod_reset(muscod_obs_, time);
  }

  out->resize(outputs_);

  if (verbose_)
  {
    std::cout << "observation state: [ ";
    std::copy(in.begin(), in.end(),
              std::ostream_iterator<double>(std::cout, " "));
    std::cout << "]" << std::endl;
  }

  // Convert MPRL states into MUSCOD states
  so_convert_obs_for_muscod(in, muscod_obs_);

  if (verbose_)
  {
    std::cout << "time: [ " << time << " ];  ";
    std::cout << "state: [ ";
    std::copy(muscod_obs_.begin(), muscod_obs_.end(),
              std::ostream_iterator<double>(std::cout, " "));
    std::cout << "]" << std::endl;
  }

  // Obtain feedback, provide new state and unblock thread
  pthread_mutex_lock(&mutex_);
  if (iv_ready_)
  {
    // Obtain feedback
    for (int IMSN = 0; IMSN < NMSN_; ++IMSN)
       muscod_action_[IMSN] = data_.qc[IMSN];

    if (verbose_)
    {
      std::cout << "Obtained Control - 1:";
      for (int IMSN = 0; IMSN < 10; ++IMSN)
        print_array(&muscod_action_[IMSN][0], NU_);
      std::cout << std::endl;
    }

    // Provide state and time
    for (int IXD = 0; IXD < NXD_; ++IXD)
      data_.initial_sd[IXD] = muscod_obs_[IXD];
    for (int IP = 0; IP < NP_; ++IP)
      data_.initial_pf[IP] = time;

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
      for (int IMSN = 0; IMSN < NMSN_; ++IMSN)
         muscod_action_[IMSN] = data_.qc[IMSN];
    }

    if (verbose_)
    {
      std::cout << "Obtained Control:";
      for (int IMSN = 0; IMSN < 10; ++IMSN)
        print_array(&muscod_action_[IMSN][0], NU_);
      std::cout << std::endl;
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
    {
      std::cout << "Feedback Control:";
      print_array(&muscod_action_[qc_cnt_][0], NU_);
      std::cout << std::endl;
    }
    std::copy(muscod_action_[qc_cnt_].begin(), muscod_action_[qc_cnt_].end(), out->begin() );
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



