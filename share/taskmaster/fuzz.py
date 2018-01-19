# Example for testing multiple configurations in parallel with Taskmaster
# See https://github.com/dcramer/taskmaster
# Run with 'tm-run fuzz.py 2 cfg=experiments.yaml'
# NOTE: Assumes the grl deployer (grld) can be found on the path

import yaml, collections, os, glob, copy, string

_mapping_tag = yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG

def dict_representer(dumper, data):
  return dumper.represent_dict(data.iteritems())
    
def dict_constructor(loader, node):
  return collections.OrderedDict(loader.construct_pairs(node))
        
yaml.add_representer(collections.OrderedDict, dict_representer)
yaml.add_constructor(_mapping_tag, dict_constructor)

def set(conf, item, value):
  # Strip leading /
  if item[0] == '/':
    item = item[1:]
    
  path = item.split('/')
  if len(path) == 1:
    conf[path[0]] = value
  else:
    set(conf[path[0]], '/'.join(path[1:]), value)
    
  return conf

def merge(base, new):
  if isinstance(base,dict) and isinstance(new,dict):
    for k,v in new.iteritems():
      if k not in base:
        base[k] = v
      else:
        base[k] = merge(base[k],v)

  return base

def get_jobs(last=None, cfg='experiments.yaml'):
  if last != None:
    print "ERROR: cannot resume"
    return
    
  stream = file(cfg, 'r')
  spec = yaml.load(stream)
  stream.close()
  
  # Iterate over experiments
  for exp in spec:
    if exp["file"] == "all":
      # All combinations of agents and environments in current directory
      for env in glob.glob('env_*.yaml'):
        for agent in glob.glob('agent_*.yaml'):
          stream = file(env)
          conf = yaml.load(stream)
          stream.close()
          stream = file(agent)
          merge(conf, yaml.load(stream))
          for job in get_experiment_jobs(os.path.basename(env)[4:-5] + "_" + os.path.basename(agent)[6:-5], conf, exp):
            yield job
    else:
      # Specific file, possibly with wildcards
      for f in glob.glob(exp["file"]):
        stream = file(f)
        conf = yaml.load(stream)
        stream.close()
        for job in get_experiment_jobs(os.path.basename(f), conf, exp):
          yield job

def get_experiment_jobs(name, conf, e):
  conf = copy.deepcopy(conf)
  indices = [0] * len(e["parameters"])
  
  # Iterate over variable combinations
  while True:
    # Construct configuration for this combination
    for c in e["configuration"]:
      value = c["value"]
      value = value.replace("$@", name)
      for i in range(len(e["parameters"])):
        value = value.replace("$%d" % i, str(e["parameters"][i]["values"][indices[i]]))
      set(conf, c["name"], value)
      
    for i in range(len(e["parameters"])):
      p = e["parameters"][i]
      set(conf, p["name"], p["values"][indices[i]])
      
    # Enqueue job
    yield copy.deepcopy(conf)
    
    # Increment
    for i in range(len(e["parameters"])):
      indices[i] = indices[i] + 1
      if indices[i] == len(e["parameters"][i]["values"]):
        indices[i] = 0
      else:
        break
    
    # Stop when we're back at the beginning
    if cmp(indices, [0] * len(indices)) == 0:
      break
  
def handle_job(conf):
    # Run
    print conf["experiment"]["output"]
    
    tmp = "/tmp/grl." + str(os.getpid()) + ".yaml"
    outfile = file(tmp, 'w')
    yaml.dump(conf, outfile)
    outfile.close()
    os.system("grld " + tmp)
    os.remove(tmp)
