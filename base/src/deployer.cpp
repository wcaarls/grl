#include <glob.h>
#include <dlfcn.h>

#include <grl/configurable.h>
#include <grl/experiment.h>

using namespace grl;
using namespace std;

void loadPlugins(const char *pattern)
{
  glob_t globbuf;
  
  printf("Looking for plugins in '%s'\n", pattern);
  
  glob(pattern, 0, NULL, &globbuf);
  for (int ii=0; ii < globbuf.gl_pathc; ++ii)
  { 
    printf("Loading plugin '%s'\n", globbuf.gl_pathv[ii]);
    if (!dlopen(globbuf.gl_pathv[ii], RTLD_NOW|RTLD_LOCAL))
      fprintf(stderr, "Error loading plugin '%s': %s\n", globbuf.gl_pathv[ii], dlerror());
  } 
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    cerr << "Usage:" << endl
         << "  " << argv[0] << " <yaml file>" << endl;
    return 1;
  }
  
  srand48(time(NULL));
  
  // Load plugins
  loadPlugins("libaddon*.so");
  
  Configuration config, task_spec;
  YAMLConfigurator configurator;
  
  task_spec.set("observation_min", VectorConstructor(0., -12*M_PI));
  task_spec.set("observation_max", VectorConstructor(2*M_PI, 12*M_PI));
  task_spec.set("action_min", VectorConstructor(-3.));
  task_spec.set("action_max", VectorConstructor(3.));
  
  configurator.populate(task_spec);
  
  Configurable *obj = configurator.load(argv[1], &config);
  Experiment *experiment = dynamic_cast<Experiment*>(obj);
  
  if (!experiment)
  {
    cerr << "Configuration root must specify an experiment" << std::endl;
    return 1;
  }
  
  experiment->run();
  
  return 0;
}
