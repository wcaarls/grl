# Example for testing multiple configurations in parallel with Taskmaster
# Runs all combinations of agents and environments in a directory
# See https://github.com/dcramer/taskmaster
# Run with 'tm-run deploy-all.py 2 dir=cfg_dir
# NOTE: Assumes the grl deployer (grld) can be found on the path

import os, glob

def get_jobs(last=None, dir='.'):
  if last != None:
    print "ERROR: cannot resume"
    return

  for e in glob.glob(os.path.join(dir, 'env_*.yaml')):
    for a in glob.glob(os.path.join(dir, 'agent_*.yaml')):
      yield "output_file:" + os.path.basename(e)[4:-5] + "_" + os.path.basename(a)[6:-5] + " " + e + " " + a
  
def handle_job(f):
  os.system('grld ' + f)
  