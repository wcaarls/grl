#!/usr/bin/python

import yaml
import sys,tty,termios
import readline

KEY_ENTER = 13
KEY_UP = 256
KEY_DOWN = 257
KEY_LEFT = 258
KEY_RIGHT = 259
KEY_UNKNOWN = 511

ANSI_UP = chr(27) + '[A'
ANSI_CLEARLINE = chr(27) + '[K'

import yaml
import yaml.constructor

try:
    # included in standard lib from Python 2.7
    from collections import OrderedDict
except ImportError:
    # try importing the backported drop-in replacement
    # it's available on PyPI
    from ordereddict import OrderedDict

# https://gist.github.com/enaeseth/844388
class OrderedDictYAMLLoader(yaml.Loader):
    """
    A YAML loader that loads mappings into ordered dictionaries.
    """

    def __init__(self, *args, **kwargs):
        yaml.Loader.__init__(self, *args, **kwargs)

        self.add_constructor(u'tag:yaml.org,2002:map', type(self).construct_yaml_map)
        self.add_constructor(u'tag:yaml.org,2002:omap', type(self).construct_yaml_map)

    def construct_yaml_map(self, node):
        data = OrderedDict()
        yield data
        value = self.construct_mapping(node)
        data.update(value)

    def construct_mapping(self, node, deep=False):
        if isinstance(node, yaml.MappingNode):
            self.flatten_mapping(node)
        else:
            raise yaml.constructor.ConstructorError(None, None,
                'expected a mapping node, but found %s' % node.id, node.start_mark)

        mapping = OrderedDict()
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            try:
                hash(key)
            except TypeError, exc:
                raise yaml.constructor.ConstructorError('while constructing a mapping',
                    node.start_mark, 'found unacceptable key (%s)' % exc, key_node.start_mark)
            value = self.construct_object(value_node, deep=deep)
            mapping[key] = value
        return mapping

# http://stackoverflow.com/questions/22397289
def getch():
  """Get a character from standard input, translating specials."""
  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)
  try:
    tty.setraw(sys.stdin.fileno())
    ch = ord(sys.stdin.read(1))
    if ch == 3:
      raise KeyboardInterrupt()
    elif ch == 27:
      ch2 = sys.stdin.read(2)
      special = ord(ch2[1])

      if special == 65:
        ch = KEY_UP
      elif special == 66:
        ch = KEY_DOWN
      elif special == 67:
        ch = KEY_LEFT
      elif special == 68:
        ch = KEY_RIGHT
      else:
        ch = KEY_UNKNOWN
  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  return ch

# http://stackoverflow.com/questions/2533120
def rlinput(prompt, prefill=''):
  """Ask for user input, prefilling with some default."""
  readline.set_startup_hook(lambda: readline.insert_text(prefill))
  try:
     return raw_input(prompt)
  finally:
     readline.set_startup_hook()

def isobject(type):
  """Returns true if the type is not a builtin type."""
  if type in ['int','double','string','vector']:
    return False
  else:
    return True

def findrequests(type):
  """Find parameter requests that match a certain type."""
  matches = list()
  
  for key in requests:
    if key[0:len(type)] == type:
      matches.append(key)
  
  return matches

def findparams(type):
  """Find registered parameters that match a certain type."""
  matches = list()

  for key in params:
    if params[key][0:len(type)] == type:
      matches.append(key)

  return matches

def selectlist(pstr, desc, options):
  """Asks the user to select from a list of options."""
  inkey = 0
  selection = 0
  while not inkey == KEY_ENTER:
    print desc + pstr + options[selection] + ANSI_CLEARLINE + pstr,
    inkey = getch()
    if inkey == KEY_UP:
      selection = max(selection-1, 0)
    elif inkey == KEY_DOWN:
      selection = min(selection+1, len(options)-1)

  return selection

def select(param, spec, path=[]):
  """Asks the user to select a value for a parameter."""
  indent = 2*len(path)

  # Set up request and description texts
  pstr = '\r' + ''.ljust(indent) + param + ': '
  desc = '\n' + ''.ljust(indent) + spec["description"] + ANSI_CLEARLINE + ANSI_UP + '\r'
  
  if isobject(spec["type"]):
    # Object
    
    # Find registered parameters matching this type
    matches = findparams(spec["type"])

    # Ask to choose between registered parameters, or create a new object
    options = list()
    if spec["optional"]:
      options.append('<default>')
    options.append('<new>')
    options.extend(matches)
    
    if len(options) > 1:
      selection = selectlist(pstr, desc, options)
    else:
      selection = 0

    if selection > spec["optional"]:
      # Chose registered parameter
      params['/'.join(path)] = params[options[selection]]
      print
      return
    elif selection == spec["optional"]:
      # Chose to create a new object
      print pstr, ANSI_CLEARLINE
      pstr = '\r' + ''.ljust(indent+2) + 'type: '
      desc = '\n' + ''.ljust(indent+2) + spec["description"] + ANSI_CLEARLINE + ANSI_UP + '\r'
        
      # Find object factories matching this type
      options = findrequests(spec["type"])

      # Should exist
      if not len(options):
        raise KeyError(spec["type"])
        
      selection = selectlist(pstr, desc, options)

      type = options[selection]
      
      # Register parameter
      params['/'.join(path)] = type
      print

      # Ask to choose values for the new object's parameters
      if requests[type]:
        for key in requests[type]:
          newpath = path[:]
          newpath.append(key)
          select(key, requests[type][key], newpath)
    else:
      # Chose not to set a value
      print '\r' + ANSI_CLEARLINE,
      return
  else:
    # Non-object type
    correct = False

    # Find registered parameters matching this type
    matches = findparams(spec["type"])

    # Ask to choose between registered parameters, default, or new value
    options = list()
    options.append('<default>')
    options.append('<new>')
    options.extend(matches)
    
    selection = selectlist(pstr, desc, options)

    if selection == 0:
      # Chose default
      print '\r', ANSI_CLEARLINE,
      return
    elif selection > 1:
      # Chose registered parameter
      params['/'.join(path)] = params[options[selection]]
      print
      return
    
    # Print description only once, not to overwrite possible error
    print desc,
    
    while correct == False:
      correct = True
  
      # Ask user input
      print '\r', ANSI_CLEARLINE,
      val = rlinput(pstr, str(spec["default"]))
    
      if spec["type"] == 'int':
        # Integer validity testing
        try:
          intval = int(val)
          if intval < spec["min"] or intval > spec["max"]:
            print ''.ljust(indent) + 'Integer value out of range', spec["min"], " - ", spec["max"], ANSI_CLEARLINE, ANSI_UP,
            correct = False
        except:
          print ''.ljust(indent) + 'Integer value expected', ANSI_CLEARLINE, ANSI_UP,
          correct = False
      elif spec["type"] == 'double':
        # Floating point validity testing
        try:
          dblval = float(val)
          if dblval < spec["min"] or dblval > spec["max"]:
            print ''.ljust(indent) + 'Double value out of range', spec["min"], " - ", spec["max"], ANSI_CLEARLINE, ANSI_UP,
            correct = False
        except:
          print ''.ljust(indent) + 'Double value expected', ANSI_CLEARLINE, ANSI_UP,
          correct = False
      elif spec["type"] == 'vector':
        # Vector validity testing
        if val[0] != '[':
          print ''.ljust(indent) + 'Vector value expected', ANSI_CLEARLINE, ANSI_UP,
          correct = False
    
    # Register parameter
    params['/'.join(path)] = spec["type"]

# Load object parameter requests, generated by requestgen
stream = file('requests.yaml', 'r')
requests = yaml.load(stream, OrderedDictYAMLLoader)
params = dict()

# Start selection from experiment
spec = {'type': 'experiment', 'description':'Experiment to run', 'optional':0}
select('experiment', spec)
