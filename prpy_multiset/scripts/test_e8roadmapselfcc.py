#!/usr/bin/python
from __future__ import print_function, unicode_literals, absolute_import, division
import atexit
import openravepy
import prpy_multiset.planning_e8roadmapselfcc

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
env.Add(robot)
robot.SetActiveManipulator('arm')
robot.SetActiveDOFs(robot.GetActiveManipulator().GetArmIndices())

robot.GetLink('wam5').Enable(False)

mug3 = env.ReadKinBodyURI("models/objects/mug3.iv")
env.Add(mug3)
mug3.SetTransform([1,0,0,0, 1,0,0])

planner = prpy_multiset.planning_e8roadmapselfcc.E8RoadmapSelfCCPlanner()
planner.roadmap_id = 'RGG(n=1000 radius=1.6 seed=1)'
#planner.CacheCalculateSave(robot)
print('self hash:', planner.GetSelfHash(robot))
