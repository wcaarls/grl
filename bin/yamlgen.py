#!/usr/bin/python

import yaml
import sys,tty,termios
import readline
from grllib import *

KEY_ENTER = 13
KEY_UP = 256
KEY_DOWN = 257
KEY_LEFT = 258
KEY_RIGHT = 259
KEY_UNKNOWN = 511

ANSI_UP = chr(27) + '[A'
ANSI_CLEARLINE = chr(27) + '[K'

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
    matches = findparams(params, spec["type"])

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
      options = findrequests(requests, spec["type"])

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
          if requests[type][key]["mutability"] == "provided":
            params['/'.join(newpath)] = requests[type][key]["type"]
          else:
            select(key, requests[type][key], newpath)
    else:
      # Chose not to set a value
      print '\r' + ANSI_CLEARLINE,
      return
  else:
    # Non-object type
    correct = False

    # Find registered parameters matching this type
    matches = findparams(params, spec["type"])

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
