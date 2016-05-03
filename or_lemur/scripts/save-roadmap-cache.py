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

parser = argparse.ArgumentParser(description='save roadmap cache')
parser.add_argument('--robot-xml') # e.g. barrettwam.robot.xml
parser.add_argument('--urdf') # can be package:// uri
parser.add_argument('--srdf') # can be package:// uri
parser.add_argument('--manip')
parser.add_argument('--roadmap-type',required=True)
parser.add_argument('--roadmap-param', action='append')
parser.add_argument('--num-batches',type=int,required=True)
args = parser.parse_args()
if args.roadmap_param is None:
   args.roadmap_param = list()

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
else:
   raise RuntimeError('either --robot-xml or --urdf and --srdf must be passed!')

# set manipulator / active dofs
if args.manip is not None:
   robot.SetActiveManipulator(args.manip)
robot.SetActiveDOFs(robot.GetActiveManipulator().GetArmIndices())

# rosrun thesis_experiments exp-multistep.py
#   --problem=workcell
#   --planner=LEMUR
#   --param=roadmap_type=CachedHaltonOffDens
#   --param=roadmap_param=num_per_batch=10000 --param=roadmap_param=gamma_factor=0.9 --param=roadmap_param=scaling=log_n --param=roadmap_param=seed=2
#   --param=max_batches=4 --param=num_batches_init=5
#   --param=do_roadmap_save=true

# create planner
with env:
   planner = openravepy.RaveCreatePlanner(env, 'LEMUR')
if planner is None:
   raise RuntimeError('planner not found!')

# create planner params
params = openravepy.Planner.PlannerParameters()
with env:
   params.SetRobotActiveJoints(robot)
params.SetInitialConfig(robot.GetActiveDOFValues())
params.SetGoalConfig(robot.GetActiveDOFValues())

# add parameters
xml = list()
if not args.roadmap_type.startswith('Cached'):
   args.roadmap_type = 'Cached{}'.format(args.roadmap_type)
xml.append('<roadmap_type>{}</roadmap_type>'.format(args.roadmap_type))
for paramstr in args.roadmap_param:
   xml.append('<roadmap_param>{}</roadmap_param>'.format(paramstr))
xml.append('<max_batches>{}</max_batches>'.format(args.num_batches))
xml.append('<num_batches_init>{}</num_batches_init>'.format(args.num_batches+1)) # a bit of a hack
xml.append('<do_roadmap_save>true</do_roadmap_save>')
params.SetExtraParameters('\n'.join(xml))

# run planner
with env:
   success = planner.InitPlan(robot, params)
   if not success:
      raise RuntimeError('InitPlan() failed!')
   planner.PlanPath(None)
