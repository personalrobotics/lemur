#!/usr/bin/env python2
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import division
import argparse
import atexit
import subprocess
import openravepy

# what resource_retriever.get_filename(url, use_protocol=False) should be!
def rr_get_filename(url):
   PACKAGE_PREFIX = 'package://'
   if not url.startswith(PACKAGE_PREFIX):
      return url
   mod_url = url[len(PACKAGE_PREFIX):]
   return subprocess.check_output(['catkin_find',mod_url]).rstrip('\n')

parser = argparse.ArgumentParser(description='family self setcache')
parser.add_argument('--robot-xml') # e.g. barrettwam.robot.xml
parser.add_argument('--urdf') # can be package:// uri
parser.add_argument('--srdf') # can be package:// uri
parser.add_argument('--manip')
args = parser.parse_args()

# load environment
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

# load robot
if args.robot_xml is not None:
   robot = env.ReadRobotXMLFile(args.robot_xml)
   env.Add(robot)
elif args.urdf is not None and args.srdf is not None:
   or_urdf = openravepy.RaveCreateModule(env, 'urdf')
   fn_urdf = rr_get_filename(args.urdf)
   fn_srdf = rr_get_filename(args.srdf)
   robot = env.GetRobot(or_urdf.SendCommand('load {} {}'.format(fn_urdf,fn_srdf)))

# set manipulator / active dofs
if args.manip is not None:
   robot.SetActiveManipulator(args.manip)
   robot.SetActiveDOFs(robot.GetActiveManipulator().GetArmIndices())

# get bounds
adofindices = robot.GetActiveDOFIndices()
lowers,uppers = robot.GetActiveDOFLimits()

# print nicely
print('Robot space bounds:')
for i,(lower,upper) in enumerate(zip(lowers,uppers)):
   joint = robot.GetJointFromDOFIndex(adofindices[i])
   print('  dof [{}] from {} to {} (joint {}, range {})'.format(
      i, lower, upper, joint.GetName(), upper-lower))

# print in bounds command line format
args = []
for i,(lower,upper) in enumerate(zip(lowers,uppers)):
   args.append('--bounds={}:{},{}'.format(i,lower,upper))

print('Bounds command line (e.g. for ompl_lemur generate-roadmap):')
print('  --dim={} {}'.format(len(adofindices), ' '.join(args)))
