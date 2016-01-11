#!/usr/bin/env python
from __future__ import print_function, unicode_literals, absolute_import, division
import openravepy
import unittest

q_start = [-2.0, -0.5, -0.2, -0.5, 0.0, 0.0, 0.0]
q_goal = [2.0,  0.5,  0.2,  0.5, 0.0, 0.0, 0.0 ]

traj_expected = [
   [ -2.00000, -0.50000, -0.20000, -0.50000,  0.00000,  0.00000,  0.00000 ],
   [ -1.38058, -0.26242, -0.88124,  0.01679, -0.04491, -0.39824,  0.55176 ],
   [ -0.64427, -0.35982, -0.09207,  1.07010,  0.25800, -0.95592, -0.21814 ],
   [ -0.03068, -0.10010, -0.30251,  1.83249, -0.29275, -0.30386, -0.07149 ],
   [  1.11981,  0.04238, -0.52874,  0.80426, -0.22390, -0.07650,  0.53342 ],
   [  2.00000,  0.50000,  0.20000,  0.50000,  0.00000,  0.00000,  0.00000 ]
]

class Wam7Testcase(unittest.TestCase):
   def wam7_test(self):
      env = openravepy.Environment()
      try:
         robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
         env.Add(robot)
         robot.SetActiveDOFs(robot.GetManipulator('arm').GetArmIndices())
         robot.SetActiveDOFValues(q_start)
         planner = openravepy.RaveCreatePlanner(env, 'E8Roadmap')
         self.assertIsNotNone(planner)
         
         params = openravepy.Planner.PlannerParameters()
         params.SetRobotActiveJoints(robot)
         params.SetGoalConfig(q_goal)
         params.SetExtraParameters(
            '<roadmap_type>Halton</roadmap_type>'
            + '<roadmap_param>num=1000</roadmap_param>'
            + '<roadmap_param>radius=2.0</roadmap_param>')
         planner.InitPlan(robot, params)
         
         traj = openravepy.RaveCreateTrajectory(env, '')
         result = planner.PlanPath(traj)
         self.assertEqual(result, openravepy.PlannerStatus.HasSolution)
         
         self.assertEqual(6, traj.GetNumWaypoints())
         for iwp in range(6):
            wp = traj.GetWaypoint(iwp)
            self.assertEqual(7, len(wp))
            for a,b in zip(wp,traj_expected[iwp]):
               self.assertAlmostEqual(a, b, delta=1.0e-5)
      
      finally:
         env.Destroy()

if __name__ == '__main__':
   openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
   unittest.main()
   openravepy.RaveDestroy()
