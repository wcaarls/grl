# Example for testing multiple configurations in parallel with Taskmaster
# See https://github.com/dcramer/taskmaster
# Run with 'tm-run deploy-all.py 2 dir=cfg_dir
# NOTE: Assumes the grl deployer (grld) can be found on the path

import os

def get_jobs(last=None, dir='.'):
  if last != None:
    print "ERROR: cannot resume"
    return

  for f in os.listdir(dir):
    if f.endswith(".yaml"):
      yield os.path.join(dir, f)
  
def handle_job(f):
  os.system('grld ' + f)
  