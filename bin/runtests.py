#!/usr/bin/python
import os, inspect, tempfile, glob, subprocess, filecmp, shutil

binpath = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.chdir(binpath + '/../tests')
tests = glob.glob('*.yaml')
tempdir = tempfile.mkdtemp()
os.chdir(tempdir)

devnull = open('/dev/null', 'w')

for test in tests:
  print "Test " + test
  
  subprocess.call([binpath + '/../build/deploy','-s','1',binpath + '/../tests/' + test], stdout=devnull)

  (root, ext) = os.path.splitext(test)
  
  for template in glob.glob(binpath + '/../tests/template/' + root + '-*.txt'):
    result = os.path.basename(template)
    
    fc = False
    try:
      fc = filecmp.cmp(result, template)
    except:
      print "Test " + test + "... failed to produce result."

    if not fc:
      print "Test " + test + "... failed."
      break

shutil.rmtree(tempdir)
  