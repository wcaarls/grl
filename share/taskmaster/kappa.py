# Example for running parameter studies with Taskmaster
# See https://github.com/dcramer/taskmaster
# Run with 'tm-run kappa.py 2'

import yaml, collections, os

_mapping_tag = yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG

def dict_representer(dumper, data):
  return dumper.represent_dict(data.iteritems())
    
def dict_constructor(loader, node):
  return collections.OrderedDict(loader.construct_pairs(node))
        
yaml.add_representer(collections.OrderedDict, dict_representer)
yaml.add_constructor(_mapping_tag, dict_constructor)
        
def get_jobs(last=None):
  if last != None:
    print "ERROR: cannot resume"
    return

  for f in [ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0 ]:
    yield f
  
def handle_job(f):
  stream = file('kappa.yaml', 'r')
  conf = yaml.load(stream)
  stream.close()
  
  output = 'kappa-' + str(f)
  
  conf['experiment']['output'] = output
  conf['experiment']['agent']['predictor']['kappa'] = f
  
  outfile = file(output + '.yaml', 'w')
  yaml.dump(conf, outfile)
  outfile.close()
  os.system('../../build/grld ' + output + '.yaml')
  