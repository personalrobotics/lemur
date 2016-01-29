#!/usr/bin/python
from __future__ import print_function, unicode_literals, absolute_import, division
import atexit
import math
import numpy
import openravepy
import prpy.planning.ompl
import prpy_lemur.planning_multiset

# start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
e = openravepy.Environment()
atexit.register(e.Destroy)

e.SetViewer('qtcoin')

# table
table = e.ReadKinBodyXMLFile('models/furniture/rolly-table.iv')
e.Add(table)
table.SetTransform([0.70711,0.70711,0,0,0,0,0])

# bottle (and its grasp)
mug = e.ReadKinBodyXMLFile('models/objects/mug3.iv')
e.Add(mug)
mug.SetTransform([1,0,0,0,0,0,0.7])
T_mug_palm = numpy.array(
   [[ 0, -1, 0, 0.000 ],
   [ 0, 0, -1, 0.075 ],
   [ 1, 0, 0, 0.100 ],
   [ 0, 0, 0, 1 ]])

# robot
r = e.ReadRobotXMLFile('robots/barrettwam.robot.xml')
e.Add(r)
r.SetTransform([0.70711,0,0.70711,0,-1.0,0,1.0])

# set up active manip, active dofs
r.SetActiveManipulator('arm')
m = r.GetActiveManipulator()
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(r,
   iktype=openravepy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
   ikmodel.autogenerate()
r.SetActiveDOFs(m.GetArmIndices())
T_palm_ee = numpy.array(
   [[ 1., 0., 0., 0. ],
    [ 0., 1., 0., 0. ],
    [ 0., 0., 1., 0.065 ],
    [ 0., 0., 0., 1. ]])
   
# get IK solution for bottle
T_ee = reduce(numpy.dot, [
   mug.GetTransform(),
   numpy.linalg.inv(T_mug_palm),
   T_palm_ee])
q_goal = m.FindIKSolution(T_ee, 0)
print('q_goal:', q_goal)

# set starting arm configuration
r.SetActiveDOFValues([2.5,-1.8,0.0,2.0,0.0,0.2,0.0])

#planner = prpy.planning.ompl.OMPLPlanner('RRTConnect')
planner = prpy_lemur.planning_multiset.MultisetPlanner()
print('outer env:', e)
t = planner.PlanToConfiguration(r, q_goal)

openravepy.planningutils.RetimeActiveDOFTrajectory(t, r)

try:
   while t is not None:
      raw_input('Press [Enter] to run the trajectory, [Ctrl]+[C] to quit ...')
      with e:
         r.GetController().SetPath(t)
except KeyboardInterrupt:
   pass

raw_input('enter to quit!')
print
