#!/usr/bin/env python2

# File: test-herb-compare.py
# Author: Chris Dellin <cdellin@gmail.com>
# Copyright: 2014, 2015 Carnegie Mellon University
# License: None

from __future__ import print_function, unicode_literals, absolute_import, division

import argparse
import atexit
import sys
import time
import numpy
import openravepy


parser = argparse.ArgumentParser(description='HERB problem experiment script.')
parser.add_argument('--planner',required=True)
parser.add_argument('--timelimit',type=float,default=600.0)
parser.add_argument('--outfile')
parser.add_argument('--outfile-lineprefix')
parser.add_argument('--stop-after', type=int) # do only these steps
parser.add_argument('--seed', type=int, default=1)

# for planner=MultiSetPRM
parser.add_argument('--lambda', dest='lambda_', type=float)
parser.add_argument('--w-interstep', default=False, action='store_true')
parser.add_argument('--w-selfcc', default=False, action='store_true')

# for planner=OMPL_RRTConnect
parser.add_argument('--range', type=float)

args = parser.parse_args()

planners = ['MultiSetPRM', 'OMPL_RRTConnect']
if args.planner not in planners:
   raise RuntimeError('planner must be in:', planners)
if args.planner == 'MultiSetPRM':
   if args.lambda_ is None:
      raise RuntimeError('MultiSetPRM lamba parameter must be passed!')
if args.planner == 'OMPL_RRTConnect':
   if args.range is None:
      raise RuntimeError('OMPL_RRTConnect range parameter must be passed!')


# hardcoded poses (openrave order, qw qx qy qz x y z)
R12 = numpy.sqrt(0.5)
T = lambda pose: openravepy.matrixFromPose(pose)
T_r          = T([ R12, 0.,  0., -R12, -0.3975, 2.38, 0.     ])
T_table      = T([ R12, 0.,  0., -R12, -0.3975, 1.61, 0.     ])
T_bin        = T([ R12, 0.,  0., -R12, -1.1,    2.3,  0.     ])
T_ee_palm    = T([ 1.,  0.,  0.,  0.,   0.,     0.,   0.1365 ])
T_mug_grasp1 = T([ 0.5,-0.5,-0.5, 0.5,  0.15,   0.,   0.09   ])
T_mugT       = T([ 0.,  0.,  0.,  1.,  -0.3975, 1.61, 0.736  ]) # table
T_mugD       = T([-R12, 0.,  0.,  R12, -1.1,    2.3,  0.0    ]) # bin
T_mug_drop   = T([-R12, 0.,  0.,  R12, -1.1,    2.3,  0.735  ]) # drop location
# robot dof values
r_dofvals = [
   5.759, -1.972, -0.22, 1.9, 0., 0., 0., 1.3,1.3,1.3,0., # right
   0.630, -1.900,  0.15, 1.9, 0., 0., 0., 2.3,2.3,2.3,0.  # left
]

# create an environment, load the robot (wam)
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
e = openravepy.Environment()
atexit.register(e.Destroy)

e.GetCollisionChecker().SetCollisionOptions(
   openravepy.CollisionOptions.ActiveDOFs)

#e.SetViewer('qtcoin')

# load a robot, ik solver
r = e.ReadRobotXMLFile('robots/herb2_padded_nosensors.robot.xml')
e.Add(r)
r.SetTransform(T_r)
r.SetDOFValues(r_dofvals,range(len(r_dofvals)))
r.SetActiveManipulator('right_wam')
r.SetActiveDOFs(r.GetActiveManipulator().GetArmIndices())
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(r,
   iktype=openravepy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
   ikmodel.autogenerate()

# HACK
for joint in r.GetJoints():
   joint.SetResolution(0.05)

# do some sweet stuff
if False:
   m = openravepy.RaveCreateModule(e, 'SubsetManager')
   e.Add(m, False, 'ssm')
   m.SendCommand('TagCurrentSubset {} selfcc true'.format(r.GetName()))
   p = openravepy.RaveCreatePlanner(e, 'MultiSetPRM')
   p.SendCommand('UseSubsetManager ssm')
   pp_self = openravepy.Planner.PlannerParameters()
   pp_self.SetExtraParameters('<eval_subgraphs>1</eval_subgraphs>')
   p.InitPlan(r, pp_self)
   t = openravepy.RaveCreateTrajectory(e, '')
   p.PlanPath(t)
   p.SendCommand('CacheSetLocation mycache')
   p.SendCommand('CacheSave')
   print('bailing super early!')
   exit()

# add fixed objects (kitchen, table, bin)
kbs = {}
kbs['kitchen'] = e.ReadKinBodyXMLFile('environments/pr_kitchen.kinbody.xml')
kbs['table'] = e.ReadKinBodyXMLFile('objects/furniture/table_zup.kinbody.xml')
kbs['bin'] = e.ReadKinBodyXMLFile('objects/household/recyclingbin-zlevel.kinbody.xml')
kbs['mug'] = e.ReadKinBodyXMLFile('objects/household/mug2.kinbody.xml')
for name,kb in kbs.items():
   if not kb:
      raise RuntimeError('kinbody {} not found!'.format(name))
   e.Add(kb)
kbs['table'].SetTransform(T_table)
kbs['bin'].SetTransform(T_bin)
kbs['mug'].SetTransform(T_mugT)

#r.SetActiveDOFValues([ 4.46478,  -0.594221, -0.3,       1.5091,   -2.46285,  -0.849687, -0.678063])
#r.Grab(kbs['mug'])
#raw_input('press enter to quit!')
#exit()

# returns a list of goals
# guaranteed to be called in sequence

def step1():
   #r.SetDOFValues([0.,0.,0.,0.],[7,8,9,10]) # open
   # get iks for mug on table (mugiksT)
   H = reduce(numpy.dot,
      [kbs['mug'].GetTransform(),
      T_mug_grasp1,
      numpy.linalg.inv(T_ee_palm)])
   return r.GetActiveManipulator().FindIKSolutions(H, openravepy.IkFilterOptions.CheckEnvCollisions)

def step2():
   # get iks for mug at drop location (mugiksdrop)
   with kbs['mug']:
      kbs['mug'].SetTransform(T_mug_drop)
      H = reduce(numpy.dot,
         [kbs['mug'].GetTransform(),
         T_mug_grasp1,
         numpy.linalg.inv(T_ee_palm)])
      goals = r.GetActiveManipulator().FindIKSolutions(H, openravepy.IkFilterOptions.CheckEnvCollisions)
   # grab the mug
   kbs['mug'].SetTransform(reduce(numpy.dot,(
      r.GetActiveManipulator().GetEndEffectorTransform(),
      T_ee_palm,
      numpy.linalg.inv(T_mug_grasp1)
      )))
   #r.SetDOFValues([1.5,1.5,1.5,0.],[7,8,9,10]) # closed
   r.Grab(kbs['mug'])
   return goals

def step3():
   r.Release(kbs['mug'])
   kbs['mug'].SetTransform(T_mugD)
   #r.SetDOFValues([0.,0.,0.,0.],[7,8,9,10]) # open
   return [[5.759, -1.972, -0.22, 1.9, 0., 0., 0.]]

steps = [step1, step2, step3]
if args.stop_after is not None:
   steps = steps[0:args.stop_after]
times = []
trajs = []

p = openravepy.RaveCreatePlanner(e, args.planner)

for i,step in enumerate(steps):
   
   goals = step()
   start = r.GetActiveDOFValues()
   
   if args.planner == 'MultiSetPRM':

      # set up the ssm
      if i==0 or not args.w_interstep:

         for m in e.GetModules():
            if m.GetXMLId() == 'SubsetManager':
               e.Remove(m)
         
         m = openravepy.RaveCreateModule(e, 'SubsetManager')
         e.Add(m, False, 'ssm')
         p = openravepy.RaveCreatePlanner(e, args.planner) # reset planner
         #p.SendCommand('SetRoadmap class=RoadmapSampledDensified seed=1 batch_n=1000 gamma_rel=1.1')
         p.SendCommand('SetRoadmap class=RoadmapSampledConst seed={} batch_n=1000 radius=2'.format(args.seed))
         p.SendCommand('UseSubsetManager ssm')
         
         if args.w_selfcc:
            for name,kb in kbs.items():
               kb.Enable(False)
            m.SendCommand('TagCurrentSubset {} selfcc true'.format(r.GetName()))
            for name,kb in kbs.items():
               kb.Enable(True)

      m.SendCommand('TagCurrentSubset {} step{} true'.format(r.GetName(),i+1))
   
   pp = openravepy.Planner.PlannerParameters()
   pp.SetRobotActiveJoints(r)
   vgoalconfig = []
   for goal in goals:
      vgoalconfig.extend(goal)
   pp.SetGoalConfig(vgoalconfig)
   if args.planner == 'MultiSetPRM':
      params = ''
      params += '<timelimit>{}</timelimit>'.format(args.timelimit)
      params += '<lambda>{}</lambda>'.format(args.lambda_)
      params += '<interroot_radius>2.0</interroot_radius>'
      pp.SetExtraParameters(params)
   elif args.planner == 'OMPL_RRTConnect':
      params = ''
      params += '<time_limit>{}</time_limit>'.format(args.timelimit)
      params += '<range>{}</range>'.format(args.range)
      if i==0:
         params += '<seed>{}</seed>'.format(args.seed)
      pp.SetExtraParameters(params)
   else:
      raise RuntimeError('unknown planner!')
   
   p.InitPlan(r,pp)
   
   if args.planner == 'MultiSetPRM' and args.w_selfcc:
      p.SendCommand('CacheSetLocation mycache')
      p.SendCommand('CacheLoad 1')
   
   t = openravepy.RaveCreateTrajectory(e, '')
   tic = time.time()
   p.PlanPath(t)
   toc = time.time()
   times.append(toc - tic)
   trajs.append(t)
   
   q_last = t.GetWaypoint(t.GetNumWaypoints()-1)
   print('setting config to:', map(float,q_last))
   r.SetActiveDOFValues(q_last)

# compute lengths
traj_lens = []
for traj in trajs:
   len_rad = 0.0
   for i in range(1,traj.GetNumWaypoints()):
      va = traj.GetWaypoint(i-1)
      vb = traj.GetWaypoint(i)
      len_rad += numpy.linalg.norm(va - vb)
   traj_lens.append(len_rad)

print('times:', times)
print('traj lens:', traj_lens)
if args.outfile is not None:
   fp = open(args.outfile, 'a')
   lines = []
   lines.append('times {}'.format(' '.join(map(str,times))))
   lines.append('lens {}'.format(' '.join(map(str,traj_lens))))
   for line in lines:
      if args.outfile_lineprefix is not None:
         fp.write(args.outfile_lineprefix)
      fp.write(line)
      fp.write('\n')
   fp.close()
