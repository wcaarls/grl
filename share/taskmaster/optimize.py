# Example for testing multiple configurations in parallel with Taskmaster
# See https://github.com/dcramer/taskmaster
# Run with 'tm-run fuzz.py 2 cfg=experiments.yaml'
# NOTE: Assumes the grl deployer (grld) can be found on the path

import yaml, collections, sys, copy, string, server, math

srv = server.Server()
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

def line_search(conf, param, values, repetitions):
  print "Optimizing ", param

  vworkers = {}
  for v in values:
    newconf = copy.deepcopy(conf)
    set(newconf, param, v)
    confstr = yaml.dump(conf)
    
    vworkers[v] = [srv.submit(confstr) for r in range(repetitions)]
    
    print "Submitted ", v
  
  vresults = {}
  for value, workers in vworkers.iteritems():
    results = [server.read(w) for w in workers]
    
    print value, results
    
    avg = sum(results)/len(results)
    stddev = math.sqrt(sum([(r-avg)**2 for r in results])/(len(results)-1))
    stderr = stddev/math.sqrt(len(results))

    vresults[value] = (avg, stddev, stderr)
    
  return vresults
  
def optimize(cfg):
  stream = open(cfg, 'r')
  spec = yaml.load(stream)
  stream.close()
  
  file = spec["file"]
  stream = open(file, 'r')
  conf = yaml.load(stream)
  stream.close()
  
  while True:
    for p in spec["parameters"]:
      results = line_search(conf, p["name"], p["values"], spec["repetitions"])
      
      print p["name"], results
      
      means = [avg for (avg, stddev, stderr) in results.values()]
      best = [v for v, (avg, stddev, stderr) in results.items() if avg == max(means)]
    
      set(conf, p["name"], best[0])
      
      print yaml.dump(conf)
    
    # Stopping criterion?

if __name__ == '__main__':
  if len(sys.argv) > 1:
    optimize(sys.argv[1])
  else:
    optimize('optimize.yaml')
