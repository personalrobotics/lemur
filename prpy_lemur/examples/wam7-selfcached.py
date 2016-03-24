#!/usr/bin/env python2
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import division
import argparse
import atexit
import openravepy
import prpy_lemur

parser = argparse.ArgumentParser()
parser.add_argument('--generate', default=False, action='store_true')
args = parser.parse_args()

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
env.Add(robot)
robot.SetActiveDOFs(robot.GetManipulator('arm').GetArmIndices())

q_start = [-2.0, -0.5, -0.2, -0.5, 0.0, 0.0, 0.0 ]
q_goal  = [ 2.0,  0.5,  0.2,  0.5, 0.0, 0.0, 0.0 ]

robot.SetActiveDOFValues(q_start)

planner = prpy_lemur.LEMURSelfCachedPlanner(
   roadmap=prpy_lemur.roadmaps.Halton(num=1000, radius=2.0))

if args.generate:
   planner.Generate(robot, 1)
else:
   traj = planner.PlanToConfiguration(robot, q_goal, max_batches=1)
   print('output traj has {} waypoints!'.format(traj.GetNumWaypoints()))
