#!/usr/bin/env python
from __future__ import print_function, unicode_literals, absolute_import, division
import atexit
import sys
import openravepy
import prpy_multiset.planning_e8roadmapselfcc

import unittest

class HashesTestcase(unittest.TestCase):
   def hashes_test(self):
      env = openravepy.Environment()
      try:
         robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
         env.Add(robot)
         robot.SetActiveDOFs(robot.GetManipulator('arm').GetArmIndices())
         planner = prpy_multiset.planning_e8roadmapselfcc.E8RoadmapSelfCCPlanner()
         planner.roadmap_id = 'RGG(n=1000 radius=2.0 seed=1)'
         selfhash = planner.GetSelfHash(robot)
         self.assertEquals('8eee198040256418c9e1613b79cba6ff', selfhash)
      finally:
         env.Destroy()

if __name__ == '__main__':
   openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
   unittest.main()
   openravepy.RaveDestroy()
