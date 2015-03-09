import yaml
import yaml.constructor
import itertools

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
