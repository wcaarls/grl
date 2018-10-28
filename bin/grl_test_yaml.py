import sys
import time
import yaml
from collections import OrderedDict

# Assumes this script is being run from grl/bin
sys.path.append('../build')

import grlpy

# Make sure YAML files are loaded in-order
_mapping_tag = yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG

def dict_representer(dumper, data):
  return dumper.represent_dict(data.items())
    
def dict_constructor(loader, node):
  return OrderedDict(loader.construct_pairs(node))
        
yaml.add_representer(OrderedDict, dict_representer)
yaml.add_constructor(_mapping_tag, dict_constructor)

# Load configuration as Python dict
stream = open("../cfg/pendulum/sarsa_tc.yaml")
yamlconf = yaml.load(stream)
stream.close()

# Random change
#yamlconf["visualization"]["points"] = 256

grlpy.verbosity(9)
# Read configuration from string
conf = grlpy.Configurator(yaml.dump(yamlconf))

# Instantiate configuration (construct objects)
inst = conf.instantiate()

# Get reference to experiment
experiment = grlpy.Experiment(inst["experiment"])

# Run
grlpy.verbosity(9)
experiment.run()
