#!/usr/bin/env python
from __future__ import print_function, unicode_literals, division, absolute_import
import atexit
import math
import openravepy
import networkx

def Halton(primes, n, radius, bounds_min, bounds_max):
   g = networkx.Graph()
   for ivertex in range(n):
      state = []
      for prime,bound_min,bound_max in zip(primes,bounds_min,bounds_max):
         sample = 0.0
         denom = prime
         index = ivertex + 1
         while 0 < index:
            sample += (index % prime) / denom
            index //= prime
            denom *= prime
         state.append(bound_min + (bound_max-bound_min) * sample)
      neighbors = []
      for ineighbor,attrs in g.nodes(data=True):
         nstate = attrs['state']
         dist2 = 0.0
         for a,b in zip(state,nstate):
            dist2 += (a - b)**2
         if math.sqrt(dist2) < radius:
            neighbors.append(ineighbor)
      g.add_node(ivertex, state=state)
      for ineighbor in neighbors:
         g.add_edge(ivertex, ineighbor)
   return g

def graph_statestr(g):
   gstr = networkx.Graph()
   for ivertex,attrs in g.nodes(data=True):
      gstr.add_node(ivertex, state=' '.join(map(str,attrs['state'])))
   for ia,ib in g.edges():
      gstr.add_edge(ia,ib)
   return gstr

q_start = [-2.0, -0.5, -0.2, -0.5, 0.0, 0.0, 0.0]
q_goal = [2.0,  0.5,  0.2,  0.5, 0.0, 0.0, 0.0 ]

print('initializing environment ...')
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

print('loading robot ...')
robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
env.Add(robot)
robot.SetActiveDOFs(robot.GetManipulator('arm').GetArmIndices())
robot.SetActiveDOFValues(q_start)
bounds_min, bounds_max = robot.GetActiveDOFLimits()

print('generate roadmap using networkx to file "nx.xml" ...')
primes7 = [2, 3, 5, 7, 11, 13, 17]
g = Halton(primes7, 1000, 2.0, bounds_min, bounds_max)
networkx.write_graphml(graph_statestr(g), 'nx.xml')

print('initializing planner ...')
planner = openravepy.RaveCreatePlanner(env, 'E8Roadmap')
params = openravepy.Planner.PlannerParameters()
params.SetRobotActiveJoints(robot)
params.SetGoalConfig(q_goal)
params.SetExtraParameters(
   '<roadmap_id>FromFile(filename=nx.xml root_radius=2.0)</roadmap_id>')
planner.InitPlan(robot, params)

print('planning path ...')
traj = openravepy.RaveCreateTrajectory(env, '')
result = planner.PlanPath(traj)
if result != openravepy.PlannerStatus.HasSolution:
   raise RuntimeError('planning failed!')

print('trajectory has {} waypoints.'.format(traj.GetNumWaypoints()))

