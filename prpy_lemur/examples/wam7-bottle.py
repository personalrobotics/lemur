#!/usr/bin/python
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import division
import atexit
import math
import numpy
import openravepy
import prpy.planning.ompl
import prpy_lemur

# start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

env.SetViewer('qtcoin')

with env:
   
   # table
   table = env.ReadKinBodyXMLFile('models/furniture/rolly-table.iv')
   env.Add(table)
   table.SetTransform([0.70711,0.70711,0,0,0,0,0])

   # bottle (and its grasp)
   mug = env.ReadKinBodyXMLFile('models/objects/mug3.iv')
   env.Add(mug)
   mug.SetTransform([1,0,0,0,0,0,0.7])
   T_mug_palm = numpy.array(
      [[ 0.,-1., 0., 0.000 ],
       [ 0., 0.,-1., 0.075 ],
       [ 1., 0., 0., 0.100 ],
       [ 0., 0., 0., 1.    ]])

   # robot
   robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
   env.Add(robot)
   robot.SetTransform([0.70711,0,0.70711,0,-1.0,0,1.0])

   # set up active manip, active dofs
   robot.SetActiveManipulator('arm')
   manip = robot.GetActiveManipulator()
   ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,
      iktype=openravepy.IkParameterization.Type.Transform6D)
   if not ikmodel.load():
      ikmodel.autogenerate()
   robot.SetActiveDOFs(manip.GetArmIndices())
   T_palm_ee = numpy.array(
      [[ 1., 0., 0., 0.    ],
       [ 0., 1., 0., 0.    ],
       [ 0., 0., 1., 0.065 ],
       [ 0., 0., 0., 1.    ]])
      
   # get IK solution for bottle
   T_ee = reduce(numpy.dot, [
      mug.GetTransform(),
      numpy.linalg.inv(T_mug_palm),
      T_palm_ee])
   q_goal = manip.FindIKSolution(T_ee, 0)
   print('q_goal:', q_goal)

   # set starting arm configuration
   robot.SetActiveDOFValues([2.5,-1.8,0.0,2.0,0.0,0.2,0.0])

   planner = prpy.planning.ompl.OMPLPlanner('RRTConnect')
   #planner = prpy_lemur.LEMURPlanner()
   
   traj = planner.PlanToConfiguration(robot, q_goal,
      roadmap=prpy_lemur.roadmaps.Halton(num=10000, radius=2.0))

openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot)

try:
   while traj is not None:
      raw_input('Press [Enter] to run the trajectory, [Ctrl]+[C] to quit ...')
      with env:
         robot.GetController().SetPath(traj)
except KeyboardInterrupt:
   print()
