/** \file deployer.cpp
 * \brief Deployer for object configurations specified through YAML.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#include <unistd.h>
#include <signal.h>
#include <string.h>

#include <grl/grl.h>
#include <grl/configurable.h>
#include <grl/experiment.h>

using namespace grl;
using namespace std;

static YAMLConfigurator *configurator = new YAMLConfigurator();
static struct sigaction act, oldact;

void reconfigure()
{
  NOTICE("Awaiting reconfiguration. Use [action] [object] [parameter=value]...");

  while (1)
  {
    Configuration config;
    std::string line;
    std::vector<std::string> words;
    getline(cin, line);
    if (line.empty())
      break;
      
    size_t seppos = 0, newpos;
    do
    {
      // Split into commands
      newpos = line.find(' ', seppos);
      std::string command = line.substr(seppos, newpos-seppos);
      
      if (!command.empty())
      {
        size_t eqpos = command.find('=');
        if (eqpos != std::string::npos)
        {
          // key=value
          std::string key = command.substr(0, eqpos);
          std::string value = command.substr(eqpos+1);
          
          config.set(key, value);
        }
        else
        {
          // action or object
          words.push_back(command);
        }
      }
      seppos = newpos+1;
    }
    while (newpos != std::string::npos);
    
    if (words.empty())
    {
      // <key=value...>
      configurator->reconfigure(config);
    }
    else if (words.size() == 1)
    {
      if (configurator->references().has(words[0]))
      {
        // object <key=value...>
        configurator->reconfigure(config, words[0]);
      }
      else
      {
        // action <key=value...>
        config.set("action", words[0]);
        configurator->reconfigure(config);
      }
    }
    else
    {
      // action object <key=value...>
      config.set("action", words[0]);
      configurator->reconfigure(config, words[1]);
    }
  }
  
  NOTICE("Reconfiguration complete");
}

void sighandler(int signum)
{
  sigaction(SIGINT, &oldact, NULL);

  reconfigure();
  
  sigaction(SIGINT, &act, NULL);
}
/*
void hSIGSEGV(int sig)
{
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}
*/

char* exe = 0;

int initialiseExecutableName()
{
    char link[1024];
    exe = new char[1024];
    snprintf(link,sizeof link,"/proc/%d/exe",getpid());
    if(readlink(link,exe,sizeof link)==-1) {
        fprintf(stderr,"ERRORRRRR\n");
        exit(1);
    }
    printf("Executable name initialised: %s\n",exe);
}

const char* getExecutableName()
{
    if (exe == 0)
        initialiseExecutableName();
    return exe;
}

/* get REG_EIP from ucontext.h */
#define __USE_GNU
#include <ucontext.h>

#if _WIN32 || _WIN64
#if _WIN64
#define ENV64
#else
#define ENV32
#endif
#endif

// Check GCC
#if __GNUC__
#if __x86_64__ || __ppc64__
#define ENV64
#else
#define ENV32
#endif
#endif

#ifdef ENV64
#define REG_EIP_RIP REG_RIP
#elif defined ENV32
#define REG_EIP_RIP REG_EIP
#endif

void bt_sighandler(int sig, siginfo_t *info,
                   void *secret) {

  void *trace[16];
  char **messages = (char **)NULL;
  int i, trace_size = 0;
  ucontext_t *uc = (ucontext_t *)secret;

  /* Do something useful with siginfo_t */
  if (sig == SIGSEGV)
    printf("Got signal %d, faulty address is %p, "
           "from %p\n", sig, info->si_addr,
           uc->uc_mcontext.gregs[REG_EIP_RIP]);
  else
    printf("Got signal %d#92;\n", sig);

  trace_size = backtrace(trace, 16);
  /* overwrite sigaction with caller's address */
  trace[1] = (void *) uc->uc_mcontext.gregs[REG_EIP_RIP];

  messages = backtrace_symbols(trace, trace_size);
  /* skip first stack frame (points here) */
  printf("[bt] Execution path:#92;\n");
  for (i=1; i<trace_size; ++i)
  {
    printf("[bt] %s#92;\n", messages[i]);

    /* find first occurence of '(' or ' ' in message[i] and assume
     * everything before that is the file name. (Don't go beyond 0 though
     * (string terminator)*/
    size_t p = 0;
    while(messages[i][p] != '(' && messages[i][p] != ' '
            && messages[i][p] != 0)
        ++p;

    char syscom[256];
    sprintf(syscom,"addr2line %p -e %.*s", trace[i] , p, messages[i] );
           //last parameter is the filename of the symbol
    system(syscom);

  }
  exit(0);
}

int main(int argc, char **argv)
{
  // signal(SIGSEGV, hSIGSEGV);
  struct sigaction sa;
  sa.sa_sigaction = bt_sighandler;
  sigemptyset (&sa.sa_mask);
  sa.sa_flags = SA_RESTART | SA_SIGINFO;
  sigaction(SIGSEGV, &sa, NULL);

  int seed = 0;
  bool user_config = false;

  int c;
  while ((c = getopt (argc, argv, "vs:c")) != -1)
  {
    switch (c)
    {
      case 'v':
        grl_log_verbosity__++;
        break;
      case 's':
        seed = atoi(optarg);
        break;
      case 'c':
        user_config = true;
        break;
      default:
        return 1;    
    }
  }

  if (optind != argc-1)
  {
    ERROR("Usage: " << endl << "  " << argv[0] << " [-v] [-c] [-s seed] [-r] [-w] [-f file] <yaml file>");
    return 1;
  }
  
  if (seed)
  {
    srand(seed);
    srand48(seed);
  }
  else
  {
    srand(time(NULL));
    srand48(time(NULL));
  }
  
  // Load plugins
  loadPlugins();
  
  Configuration config;
  
  configurator->load(argv[optind], &config);
  
  Configurable *obj = (Configurable*)config["experiment"].ptr();
  Experiment *experiment = dynamic_cast<Experiment*>(obj);
  
  if (!experiment)
  {
    ERROR("Specified experiment has wrong type");
    return 1;
  }
    
  if (user_config)
    reconfigure();
  
  bzero(&act, sizeof(struct sigaction));
  act.sa_handler = sighandler;
  act.sa_flags = SA_NODEFER;
  sigaction(SIGINT, &act, &oldact);

  NOTICE("Starting experiment");

  try
  {
    experiment->run();
  }
  catch (Exception &e)
  {
    ERROR(e.what());
    ERROR("Stack trace:\n" << e.trace());
  }
  
  NOTICE("Cleaning up");
  
  delete configurator;
  
  NOTICE("Exiting");

  return 0;
}
