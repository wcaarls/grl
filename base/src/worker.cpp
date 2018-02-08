/** \file worker.cpp
 * \brief Remote worker for deploying object configurations specified through YAML.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-02-06
 *
 * \copyright \verbatim
 * Copyright (c) 2018, Wouter Caarls
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

#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <itc/itc.h>

#include <grl/grl.h>
#include <grl/configurable.h>
#include <grl/experiment.h>

#define BUFSIZE (16*1024)

using namespace grl;
using namespace std;

int makeConnection(const char *_host)
{
  char host[256];
  strcpy(host, _host);

  char *port = strchr(host, ':');
  if (port)
    *(port++) = 0;
  else
    port = &host[strlen(host)];

  int optval = 1;
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  
  if (fd < 0)
  {
    ERROR("Could not open socket: " << strerror(errno));
    return -1;
  }
  
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval) < 0)
  {
    ERROR("Could not set socket options: " << strerror(errno));
    close(fd);
    return -1;
  }

  struct sockaddr_in addr;
  bzero((char *) &addr, sizeof(addr));
  addr.sin_family = AF_INET;

  struct hostent *server;
  server = gethostbyname(host);
  if (!server)
  {
    ERROR("Could not resolve hostname '" << host << "': " << strerror(errno));
    close(fd);
    return -1;
  }

  bcopy((char *)server->h_addr, (char *)&addr.sin_addr.s_addr, server->h_length);
  if (*port)
    addr.sin_port = htons(atoi(port));
  else
    addr.sin_port = htons(3373);

  TRACE("Connecting to " << inet_ntoa(addr.sin_addr) << ":" << ntohs(addr.sin_port));
  if (connect(fd,(struct sockaddr *) &addr,sizeof(addr)) < 0)
  {
    TRACE("Could not connect to server: " << strerror(errno));
    close(fd);
    return -1;
  }

  return fd;
}

class Worker: public itc::Thread
{
  protected:
    char host_[256];

  public:
    void setHost(char *host)
    {
      strcpy(host_, host);
    }
  
    virtual void run()
    {
      while (ok())
      {
        NOTICE("Connecting to " << host_);

        int fd;
        while ((fd = makeConnection(host_)) == -1)
          sleep(1);

        Configurator *temp=nullptr, *configurator=nullptr;
        do
        {
          NOTICE("Waiting for configuration");

          char yaml[BUFSIZE] = {0};
          size_t n=0;
          ssize_t nread;
          do
          {
            nread = read(fd, &yaml[n], BUFSIZE - n);
            n += nread;
          } while (nread && yaml[n-1]);
          
          if (!n)
          {
            WARNING("Connection closed by server");
            break;
          }

          INFO("Loading configuration from socket");

          try
          {
            temp = loadYAML("", "", nullptr, YAML::Load(yaml));
          }
          catch (std::exception &e)
          {
            ERROR(e.what());
            break;
          }

          if (!temp)
          {
            ERROR("Could not load configuration");
            break;
          }

          INFO("Instantiating configuration");
          configurator = temp->instantiate();

          if (!configurator)
          {
            ERROR("Could not instantiate configuration");
            break;
          }

          Configurator *expconf = configurator->find("experiment");

          if (!expconf)
          {
            ERROR("YAML configuration does not specify an experiment");
            break;
          }

          Configurable *obj = expconf->ptr();
          Experiment *experiment = dynamic_cast<Experiment *>(obj);

          if (!experiment)
          {
            ERROR("Specified experiment has wrong type");
            break;
          }

          INFO("Starting experiment");

          LargeVector result;
          try
          {
            result = experiment->run();
          }
          catch (Exception &e)
          {
            ERROR(e.what());
            ERROR("Stack trace:\n" << e.trace());
            break;
          }

          INFO("Writing result");

          std::ostringstream oss;
          for (size_t ii=0; ii != result.size(); ++ii)
            oss << result[ii] << std::endl;
          std::string str = oss.str();
          const char *cstr = str.c_str();
          
          if (write(fd, cstr, strlen(cstr)) != strlen(cstr))
            ERROR("Couldn't write result");
        }
        while (0);

        NOTICE("Experiment done. Cleaning up");
        safe_delete(&temp);
        safe_delete(&configurator);
        close(fd);
      }
    }
};

int main(int argc, char **argv)
{
  unsigned int seed = 0, threads = 1;

  int c;
  while ((c = getopt (argc, argv, "vs:t:")) != -1)
  {
    switch (c)
    {
      case 'v':
        grl_log_verbosity__++;
        break;
      case 's':
        seed = (unsigned int) atoi(optarg);
        break;
      case 't':
        threads = (unsigned int) atoi(optarg);
        break;
      default:
        return 1;    
    }
  }

  if (optind > argc-1)
  {
    ERROR("Usage: " << endl << "  " << argv[0] << " [-v] [-c] [-s seed] [-t threads] <host:port>");
    return 1;
  }
  
  if (seed)
  {
    srand(seed);
    srand48(seed);
  }
  else
  {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long unsigned int tseed = tv.tv_sec ^ tv.tv_usec ^ getpid();
  
    srand(tseed);
    srand48(tseed);
  }
  
  // Load plugins
  loadPlugins();
  
  Worker workers[threads];
  for (size_t ii=0; ii < threads; ++ii)
  {
    workers[ii].setHost(argv[optind]);
    workers[ii].start();
  }
    
  for (size_t ii=0; ii < threads; ++ii)
    workers[ii].join();
}
