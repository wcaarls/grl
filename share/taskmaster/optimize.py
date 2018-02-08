# Example for testing multiple configurations in parallel with Taskmaster
# See https://github.com/dcramer/taskmaster
# Run with 'tm-run fuzz.py 2 cfg=experiments.yaml'
# NOTE: Assumes the grl deployer (grld) can be found on the path

import yaml, collections, sys, copy, string, server, math, random

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

def line_search(conf, param, values, repetitions, regret):
  print "Optimizing", param

  vworkers = {}
  for v in values:
    newconf = copy.deepcopy(conf)
    set(newconf, param, v)
    confstr = yaml.dump(newconf)
    
    vworkers[v] = [srv.submit(confstr) for r in range(repetitions)]
    
    print "Submitted", v
  
  vresults = {}
  for value, workers in vworkers.iteritems():
    results = [server.read(w, regret) for w in workers]
    
    avg = sum(results)/len(results)
    stddev = math.sqrt(sum([(r-avg)**2 for r in results])/(len(results)-1))
    stderr = stddev/math.sqrt(len(results))
    
    print value, (avg, stddev, stderr)

    vresults[value] = (avg, stddev, stderr)
    
  return vresults
  
def random_optimize(cfg, spec, conf):
  print "Random search"

  bestscore = -100000
  bestout = cfg[:-5] + '-best.yaml'
  it = 0
  
  while it < spec["rounds"]:
    workers = []
    for i in range(max(1, int(100/spec["repetitions"]))):
      for p in spec["parameters"]:
        set(conf, p["name"], p["values"][random.randrange(len(p["values"]))])
      confstr = yaml.dump(conf)
      
      print "Submitting new configuration"
      workers.append((conf, [srv.submit(confstr) for r in range(spec["repetitions"])]))

    for (c, cworkers) in workers:
      results = [server.read(w, spec["regret"]) for w in cworkers]
      
      avg = sum(results)/len(results)
      stddev = math.sqrt(sum([(r-avg)**2 for r in results])/(len(results)-1))
      stderr = stddev/math.sqrt(len(results))
      
      resconf = {}
      resconf["mean"] = avg
      resconf["stddev"] = stddev
      resconf["stderr"] = stderr
      c["results"] = resconf
      
      outfile = cfg[:-5] + '-' + str(it) + '.yaml'
      stream = open(outfile, 'w')
      yaml.dump(c, stream)
      stream.close()
      
      score = avg-1.96*stderr
      
      if score > bestscore:
        print "Found new best lower confidence margin", score
        bestscore = score
        stream = open(bestout, 'w')
        yaml.dump(conf, stream)
        stream.close()

      it += 1

def line_optimize(cfg, spec, conf):
  it = 0
  
  for round in range(spec["rounds"]):
    for p in spec["parameters"]:
      results = line_search(conf, p["name"], p["values"], spec["repetitions"], spec["regret"])
      
      means = [avg for (avg, stddev, stderr) in results.values()]
      best = [v for v, (avg, stddev, stderr) in results.items() if avg == max(means)]
      
      print "Chose", best[0]
    
      set(conf, p["name"], best[0])
      
      resconf = {}
      resconf["parameter"] = p["name"]
      resconf["values"] = [v for v, (avg, stddev, stderr) in results.items()]
      resconf["mean"] = [avg for v, (avg, stddev, stderr) in results.items()]
      resconf["stddev"] = [stddev for v, (avg, stddev, stderr) in results.items()]
      resconf["stderr"] = [stderr for v, (avg, stddev, stderr) in results.items()]
      
      bestconf = {}
      bestconf["value"] = best[0]
      bestconf["mean"] = results[best[0]][0]
      bestconf["stddev"] = results[best[0]][1]
      bestconf["stderr"] = results[best[0]][2]
      
      resconf["best"] = bestconf
      conf["results"] = resconf
      
      outfile = cfg[:-5] + '-' + str(it) + '.yaml'
      stream = open(outfile, 'w')
      yaml.dump(conf, stream)
      stream.close()
      it += 1
    
    # Stopping criterion?

if __name__ == '__main__':
  cfg = 'optimize.yaml'
  if len(sys.argv) > 1:
    cfg = sys.argv[1]

  stream = open(cfg, 'r')
  spec = yaml.load(stream)
  stream.close()
  
  file = spec["file"]
  stream = open(file, 'r')
  conf = yaml.load(stream)
  stream.close()
  
  if spec["algorithm"] == 'line':
    line_optimize(cfg, spec, conf)
  elif spec["algorithm"] == 'random':
    random_optimize(cfg, spec, conf)
  else:
    raise Exception("Unknown optimization algorithm " + spec["algorithm"])
