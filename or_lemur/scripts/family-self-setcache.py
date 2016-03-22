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

# example usage:
# $ rosrun or_lemur family-self-setcache.py
#     --urdf=package://herb_description/robots/herb.urdf
#     --srdf=package://herb_description/robots/herb.srdf --manip=right
#     --roadmap-type=HaltonOffDens --roadmap-param=num_per_batch=1000
#     --roadmap-param=radius_first_batch=2.0 --roadmap-param=seed=0
#     --num-batches=1 --setcache=setcache-Self.txt

parser = argparse.ArgumentParser(description='family self setcache')
parser.add_argument('--robot-xml') # e.g. barrettwam.robot.xml
parser.add_argument('--urdf') # can be package:// uri
parser.add_argument('--srdf') # can be package:// uri
parser.add_argument('--manip')
parser.add_argument('--collision-checker')
parser.add_argument('--roadmap-type', required=True)
parser.add_argument('--roadmap-param', action='append') # values name=value
parser.add_argument('--num-batches', type=int, required=True)
parser.add_argument('--setcache', required=True)
args = parser.parse_args()

# load environment
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

# load collision checker
if args.collision_checker is not None:
   collision_checker = openravepy.RaveCreateCollisionChecker(env, args.collision_checker)
   env.SetCollisionChecker(collision_checker)

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

# disable any link that moves due to non-active dofs
print('Computing self-checked set cache for these links:')
for linkindex,link in enumerate(robot.GetLinks()):
   inactive_affected = False
   for dofindex in range(robot.GetDOF()):
      if dofindex in robot.GetActiveDOFIndices():
         continue
      joint = robot.GetJointFromDOFIndex(dofindex)
      jointindex = robot.GetJoints().index(joint)
      if robot.DoesAffect(jointindex, linkindex):
         inactive_affected = True
   if inactive_affected:
      link.Enable(False)
   else:
      print('  [{}] {} ({} geoms)'.format(
         linkindex, link.GetName(),
         len(link.GetGeometries())))


# create a family
family = openravepy.RaveCreateModule(env, 'Family')
env.Add(family, False, '--robot-name={}'.format(robot.GetName()))

# save self-check
family.SendCommand('Let Self = $live')

print('Current family:')
s = family.SendCommand('PrintCurrentFamily')
print(s)

self_header = family.SendCommand('GetHeaderFromSet Self')
print('Self header:')
print(self_header, end='')

# create a planner
planner = openravepy.RaveCreatePlanner(env, 'FamilyPlanner')

# create params
params = openravepy.Planner.PlannerParameters()
params.SetRobotActiveJoints(robot)
params.SetInitialConfig([])
params.SetGoalConfig([])
paramsxml = list()
if args.roadmap_type is not None:
   paramsxml.append('<roadmap_type>{}</roadmap_type>'.format(args.roadmap_type))
for kv in args.roadmap_param:
   paramsxml.append('<roadmap_param>{}</roadmap_param>'.format(kv))
paramsxml.append('<max_batches>{}</max_batches>'.format(args.num_batches))
paramsxml.append('<solve_all>true</solve_all>')
paramsxml.append('<family_module>{}</family_module>'.format(family.SendCommand('GetInstanceId')))
paramsxml.append('<family_setcaches><setcache><name>Self</name><filename>{}</filename></setcache></family_setcaches>'.format(args.setcache))
paramsstr = '\n'.join(paramsxml)
print('Planner parameters:')
print(paramsstr)
params.SetExtraParameters(paramsstr)

print('Calling planner.InitPlan ...')
success = planner.InitPlan(robot, params)
if not success:
   raise RuntimeError('InitPlan failed!')
print('Calling planner.PlanPath ...')

traj = openravepy.RaveCreateTrajectory(env, '')
result = planner.PlanPath(traj)
if result != openravepy.PlannerStatus.HasSolution:
   raise RuntimeError('Planning failed!')

print('Saving set cache ...')
planner.SendCommand('SaveSetCaches')
