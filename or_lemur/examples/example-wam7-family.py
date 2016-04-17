#!/usr/bin/env python2
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import division
import atexit
import numpy
import openravepy

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
env.Add(robot)
robot.SetActiveManipulator('arm')
robot.SetActiveDOFs(robot.GetActiveManipulator().GetArmIndices())

# address this problem
q_start = [-2.0, -0.5, -0.2, -0.5, 0.0, 0.0, 0.0 ]
q_goal  = [ 2.0,  0.5,  0.2,  0.5, 0.0, 0.0, 0.0 ]

family = openravepy.RaveCreateModule(env, 'Family')
env.Add(family, False, '--robot-name=BarrettWAM')

# save self-check
family.SendCommand('Let Self = $live')

# load up a box
box = openravepy.RaveCreateKinBody(env,'')
box.SetName('box')
aabbs = numpy.zeros((0,6))
aabb = numpy.array([[0,0,0, 0.1,0.1,0.1]])
aabbs = numpy.vstack((aabbs,aabb))
box.InitFromBoxes(aabbs,True)
env.Add(box)
box.SetTransform([1.,0.,0.,0., 0.,0.,0.75])

if False:
   env.SetViewer('qtcoin')
   robot.SetActiveDOFValues(q_start)
   raw_input('start')
   robot.SetActiveDOFValues(q_goal)
   raw_input('goal')
   raw_input('enter to quit')
   exit()

# save with box
family.SendCommand('Let AllBox = $live')

print('PrintCurrentFamily ...')
s = family.SendCommand('PrintCurrentFamily')
print(s)

self_header = family.SendCommand('GetHeaderFromSet Self')
print('self_header:')
print(self_header)

# create a planner
planner = openravepy.RaveCreatePlanner(env, 'FamilyPlanner')

# turn off the box, and plan
box.Enable(False)

# plan q_start -> q_goal (with no box!)
robot.SetActiveDOFValues(q_start)
params = openravepy.Planner.PlannerParameters()
params.SetRobotActiveJoints(robot)
params.SetExtraParameters('''
   <roadmap_type>RGGDens</roadmap_type>
   <roadmap>
      <num_per_batch>10000</num_per_batch>
      <radius_first_batch>2.0</radius_first_batch>
	   <seed>0</seed>
   </roadmap>
   <do_timing>true</do_timing>
   <family_module>{}</family_module>
   <family_setcaches>
      <setcache><name>Self</name><filename>setcache-Self.txt</filename></setcache>
   </family_setcaches>'''
   .format(family.SendCommand('GetInstanceId')))
params.SetGoalConfig(q_goal)
print('calling planner.InitPlan ...')
success = planner.InitPlan(robot, params)
if not success:
   raise RuntimeError('InitPlan failed!')
print('calling planner.PlanPath ...')

traj = openravepy.RaveCreateTrajectory(env, '')
result = planner.PlanPath(traj)
if result != openravepy.PlannerStatus.HasSolution:
   raise RuntimeError('planning failed!')
print('GetTimes:', planner.SendCommand('GetTimes'))

#planner.SendCommand('SolveAll')
#planner.SendCommand('CacheSaveAll')

raise RuntimeError('borking early!')

print('AGAIN!')

traj = openravepy.RaveCreateTrajectory(env, '')
result = planner.PlanPath(traj)
if result != openravepy.PlannerStatus.HasSolution:
   raise RuntimeError('planning failed!')
print('GetTimes:', planner.SendCommand('GetTimes'))

print('AGAIN, WITH BOX ON!')
box.Enable(True)

traj = openravepy.RaveCreateTrajectory(env, '')
result = planner.PlanPath(traj)
if result != openravepy.PlannerStatus.HasSolution:
   raise RuntimeError('planning failed!')
print('GetTimes:', planner.SendCommand('GetTimes'))
