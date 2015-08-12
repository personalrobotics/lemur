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
import prpy_multiset.planning_multiset

parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logfile',required=True)
args = parser.parse_args()

# start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
e = openravepy.Environment()
atexit.register(e.Destroy)

e.SetViewer('qtcoin')

m_urdf = openravepy.RaveCreateModule(e, 'urdf')

#planner = prpy.planning.ompl.OMPLPlanner('RRTConnect')
#planner = prpy.planning.CBiRRTPlanner()
planner = prpy_multiset.planning_multiset.MultisetPlanner()

# read log file
yamldict = yaml.safe_load(open(args.logfile))

# load environment
for kbdict in yamldict['environment']['kinbodies'].values():
   name = kbdict['name']
   uri = kbdict['uri']
   if kbdict['is_robot']:
      urdf,srdf = uri.split()
      robot_name = m_urdf.SendCommand('Load {:s} {:s}'.format(urdf,srdf))
      if robot_name != name:
         raise RuntimeError('error, or_urdf name mismatch!')
      kb = e.GetRobot(robot_name)
      kb.SetActiveDOFs(kbdict['robot_state']['active_dof_indices'])
      kb.SetActiveManipulator(kbdict['robot_state']['active_manipulator'])
   else:
      kb = e.ReadKinBodyXMLFile(uri)
      kb.SetName(name)
      e.Add(kb)
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
   method = None
# args
args = []
robot = e.GetRobot(yamldict['request']['args'][0])
for arg in yamldict['request']['args'][1:]:
   args.append(numpy.array(arg))
# kwargs
kwargs = yamldict['request']['kw_args']
print('args:')
print(args)
print(e.GetKinBody('herb').GetActiveManipulator())

# load ik solver for robot in case it's needed
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,
   iktype=openravepy.IkParameterizationType.Transform6D)
if not ikmodel.load():
   ikmodel.autogenerate()
   
if True:
   print('### OVERRIDING REQUEST TO PLANTOCONFIG TO REACHED POINT!')
   method = planner.PlanToConfiguration
   args = [yamldict['result']['traj_last']]
   kwargs = {}

# call planning method itself ...
t = method(robot, *args, **kwargs)

# retime/view resulting trajectory
openravepy.planningutils.RetimeActiveDOFTrajectory(t, robot)
try:
   while t is not None:
      raw_input('Press [Enter] to run the trajectory, [Ctrl]+[C] to quit ...')
      with e:
         robot.GetController().SetPath(t)
except KeyboardInterrupt:
   print()
   raw_input('enter to quit!')
   print()
