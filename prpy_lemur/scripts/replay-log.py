#!/usr/bin/python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import argparse
import atexit
import math

import numpy
import openravepy
import yaml

import prpy.planning

parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logfile',required=True)
args = parser.parse_args()

# start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

env.SetViewer('qtcoin')

#planner = prpy.planning.ompl.OMPLPlanner('RRTConnect')
#planner = prpy.planning.CBiRRTPlanner()
#planner = prpy_lemur.planning_multiset.MultisetPlanner()
planner = prpy.planning.openrave.OpenRAVEPlanner('birrt')

# read log file
yamldict = yaml.safe_load(open(args.logfile))

# load environment
for kbdict in yamldict['environment']['kinbodies'].values():
   name = kbdict['name']
   uri = kbdict['uri']
   # load/add kinbody
   if kbdict['is_robot']:
      if uri.endswith('.robot.xml'):
         kb = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
         kb.SetName(name)
         env.Add(kb)
      else:
         print('uri:', uri)
         urdf,srdf = uri.split()
         m_urdf = openravepy.RaveCreateModule(env, 'urdf')
         robot_name = m_urdf.SendCommand('Load {:s} {:s}'.format(urdf,srdf))
         if robot_name != name:
            raise RuntimeError('error, or_urdf name mismatch!')
         kb = env.GetRobot(robot_name)
      kb.SetActiveDOFs(kbdict['robot_state']['active_dof_indices'])
      kb.SetActiveManipulator(kbdict['robot_state']['active_manipulator'])
   else:
      kb = e.ReadKinBodyXMLFile(uri)
      kb.SetName(name)
      env.Add(kb)
   # transform
   txdict = kbdict['kinbody_state']['transform']
   orpose = txdict['orientation'] + txdict['position']
   kb.SetTransform(orpose)
   # dof values
   kb.SetDOFValues(kbdict['kinbody_state']['dof_values'])

# load request
try:
   method = getattr(planner, yamldict['request']['method'])
except AttributeError:
   raise RuntimeError('that planner does not request that planning method!')

# args
args = []
robot = env.GetRobot(yamldict['request']['args'][0])
for arg in yamldict['request']['args'][1:]:
   args.append(numpy.array(arg))
# kwargs
kwargs = yamldict['request']['kw_args']
print('args:', args)
print('manipulator:', robot.GetActiveManipulator())

# load ik solver for robot in case it's needed
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,
   iktype=openravepy.IkParameterizationType.Transform6D)
if not ikmodel.load():
   ikmodel.autogenerate()

# call planning method itself ...
print('calling planning method ...')
traj = method(robot, *args, **kwargs)

# retime/view resulting trajectory
openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot)
try:
   while traj is not None:
      raw_input('Press [Enter] to run the trajectory, [Ctrl]+[C] to quit ...')
      with env:
         robot.GetController().SetPath(traj)
except KeyboardInterrupt:
   print()
