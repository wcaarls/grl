import yaml
import itertools
import threading
import socket

try:
    # included in standard lib from Python 2.7
    from collections import OrderedDict
except ImportError:
    # try importing the backported drop-in replacement
    # it's available on PyPI
    from ordereddict import OrderedDict

class hashabledict(OrderedDict):
    def __hash__(self):
        return hash(yaml.dump(self))

_mapping_tag = yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG

def dict_representer(dumper, data):
  return dumper.represent_dict(data.iteritems())
    
def dict_constructor(loader, node):
  return hashabledict(loader.construct_pairs(node))
        
yaml.add_representer(hashabledict, dict_representer)
yaml.add_constructor(_mapping_tag, dict_constructor)

class Server():
  def __init__(self, port=3373):
    """Spawn thread to listen to connections"""
    self.port = port
    self.workers = []
    self.condition = threading.Condition()
    self.thread = threading.Thread(target=Server.thread, args=[self])
    self.thread.daemon = True
    self.thread.start()

  def thread(self):
    """Listen to connections, adding them to available workers"""
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', self.port))
    s.listen(100)
    
    print "Server listening for connections on port", self.port
    
    try:
      while True:
        c, _ = s.accept()
        with self.condition:
          self.workers.append(c)
          self.condition.notify()
    except e:
      raise e

  def submit(self, conf):
    """ Submit job to available worker"""
    # Wait for available worker
    with self.condition:
      while len(self.workers) == 0:
        self.condition.wait()
      w = self.workers.pop()
    
    # Send configuration
    w.send(conf + '\0')
    
    # Return stream for reading by submitter
    return w

def readWorker(w, regret):
  """Read worker result, returning either simple or cumulative regret"""
  data = [float(v) for v in w.makefile().readlines()]
  w.close()
  
  if regret == 'simple':
    sample = int(len(data)/20)
    if sample == 0:
      raise Exception("Worker did not return data")
    return sum(data[-sample:])/sample
  elif regret == 'cumulative':
    return sum(data)
  else:
    raise Exception("Unknown regret type " + regret)


def splittype(type):
  """Splits type into base and role."""
  temp = type.split('.')
  if len(temp) == 1:
    return temp[0], ''
  else:
    return temp[0], temp[1]

def isobject(type):
  """Returns true if the type is not a builtin type."""
  base, role = splittype(type)
  
  if base in ['int','double','string','vector']:
    return False
  else:
    return True

def isnumber(type):
  try:
    float(type)
    return True
  except ValueError:
    return False

def findrequests(requests, type):
  """Find parameter requests that match a certain type."""
  matches = list()
  
  base, role = splittype(type)
  
  for key in requests:
    if key[0:len(base)] == base and (role == "" or key[-len(role):] == role):
      keybase, keyrole = splittype(key)
      matches.append(keybase)
  
  return sorted(list(set(matches)))

def findparams(params, type):
  """Find registered parameters that match a certain type."""
  components = type.split('+')
  
  pmatches = list()
  
  for c in components:
    cmatches = list()
    
    if isnumber(c):
      cmatches.append(c)
    else:
      base, role = splittype(c)
      
      for key in params:
        if params[key][0:len(base)] == base and (role == "" or params[key][-len(role):] == role):
          keybase, keyrole = splittype(key)
          cmatches.append(keybase)
        
    pmatches.append(cmatches)

  # Cartesian product
  matches = list()
  for element in itertools.product(*pmatches):
    matches.append('+'.join(element))

  return sorted(list(set(matches)))

def setconf(conf, param, value):
  """Set parameter in configuration to value"""
  # Strip leading /
  if param[0] == '/':
    param = param[1:]
    
  path = param.split('/')
  if len(path) == 1:
    conf[path[0]] = value
  else:
    setconf(conf[path[0]], '/'.join(path[1:]), value)
    
  return conf

def getconf(conf, param):
  """Get parameter value in configuration"""
  # Strip leading /
  if param[0] == '/':
    param = param[1:]
    
  path = param.split('/')
  if len(path) == 1:
    return conf[path[0]]
  else:
    return getconf(conf[path[0]], '/'.join(path[1:]))

def mergeconf(base, new):
  """Merge configurations"""
  if isinstance(base,dict) and isinstance(new,dict):
    for k,v in new.iteritems():
      if k not in base:
        base[k] = v
      else:
        base[k] = merge(base[k],v)

  return base
