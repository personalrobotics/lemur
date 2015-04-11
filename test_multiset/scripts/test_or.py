#!/usr/bin/env python2

# File: test_or.py
# Author: Chris Dellin <cdellin@gmail.com>
# Copyright: 2015 Carnegie Mellon University
# License: BSD

from __future__ import print_function, unicode_literals, absolute_import, division

import atexit
import sys
import time
import numpy
import openravepy

lambda_ = 0.0001
#lambda_ = 0.5
#lambda_ = 0.9999
w_inter_step = False
w_selfcc = False

# hardcoded poses (openrave order, qw qx qy qz x y z)
R12 = numpy.sqrt(0.5)
T = lambda pose: openravepy.matrixFromPose(pose)
T_r          = T([ R12, 0.,  0., -R12, -0.3975, 2.38, 0.     ])
T_table      = T([ R12, 0.,  0., -R12, -0.3975, 1.61, 0.     ])
T_bin        = T([ R12, 0.,  0., -R12, -1.1,    2.3,  0.     ])
T_ee_palm    = T([ 1.,  0.,  0.,  0.,   0.,     0.,   0.1365 ])
T_mug_grasp1 = T([ 0.5,-0.5,-0.5, 0.5,  0.15,   0.,   0.09   ])
T_mugT       = T([ 0.,  0.,  0.,  1.,  -0.3975, 1.61, 0.735  ]) # table
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

# get iks for mug on table
kbs['mug'].SetTransform(T_mugT)
H = numpy.dot(
   numpy.dot(kbs['mug'].GetTransform(), T_mug_grasp1),
   numpy.linalg.inv(T_ee_palm)
   )
mugiksT = r.GetActiveManipulator().FindIKSolutions(H, openravepy.IkFilterOptions.CheckEnvCollisions)

# get iks for mug at drop location
kbs['mug'].SetTransform(T_mug_drop)
H = numpy.dot(
   numpy.dot(kbs['mug'].GetTransform(), T_mug_grasp1),
   numpy.linalg.inv(T_ee_palm)
   )
mugiksdrop = r.GetActiveManipulator().FindIKSolutions(H, openravepy.IkFilterOptions.CheckEnvCollisions)


# create the three problem definitions (encoded as planner parameters xmls)

def s1():
   r.Release(kbs['mug'])
   kbs['mug'].SetTransform(T_mugT)
   #r.SetDOFValues([0.,0.,0.,0.],[7,8,9,10]) # open
pp1 = openravepy.Planner.PlannerParameters()
pp1.SetExtraParameters(''
   + '<startstate>5.759 -1.972 -0.22 1.9 0. 0. 0.</startstate>\n'
   + '\n'.join(['<goalstate>{}</goalstate>'.format(' '.join(str(v) for v in q)) for q in mugiksT])
   + '<lambda>{}</lambda>'.format(lambda_)
   + '<interroot_radius>2.0</interroot_radius>'
)

def s2():
   r.Release(kbs['mug'])
   kbs['mug'].SetTransform(reduce(numpy.dot,(
      r.GetActiveManipulator().GetEndEffectorTransform(),
      T_ee_palm,
      numpy.linalg.inv(T_mug_grasp1)
      )))
   #r.SetDOFValues([1.5,1.5,1.5,0.],[7,8,9,10]) # closed
   r.Grab(kbs['mug'])
pp2 = openravepy.Planner.PlannerParameters()
pp2.SetExtraParameters(''
   + '\n'.join(['<startstate>{}</startstate>'.format(' '.join(str(v) for v in q)) for q in mugiksT])
   + '\n'.join(['<goalstate>{}</goalstate>'.format(' '.join(str(v) for v in q)) for q in mugiksdrop])
   + '<lambda>{}</lambda>'.format(lambda_)
   + '<interroot_radius>2.0</interroot_radius>'
)

def s3():
   r.Release(kbs['mug'])
   kbs['mug'].SetTransform(T_mugD)
   #r.SetDOFValues([0.,0.,0.,0.],[7,8,9,10]) # open
pp3 = openravepy.Planner.PlannerParameters()
pp3.SetExtraParameters(''
   + '\n'.join(['<startstate>{}</startstate>'.format(' '.join(str(v) for v in q)) for q in mugiksdrop])
   + '<goalstate>5.759 -1.972 -0.22 1.9 0. 0. 0.</goalstate>\n'
   + '<lambda>{}</lambda>'.format(lambda_)
   + '<interroot_radius>2.0</interroot_radius>'
)

plans = [[s1,pp1],[s2,pp2],[s3,pp3]]



times = []
trajs = []

if w_inter_step:

   m = openravepy.RaveCreateModule(e, 'SubsetManager')
   e.Add(m, False, 'ssm')
   p = openravepy.RaveCreatePlanner(e, 'MultiSetPRM')
   p.SendCommand('UseSubsetManager ssm')
   p.SendCommand('SetRoadmap class=RoadmapSampledConst seed=419884521 batch_n=1000 radius=2')

   if w_selfcc:
      for name,kb in kbs.items():
         kb.Enable(False)
      m.SendCommand('TagCurrentSubset {} selfcc true'.format(r.GetName()))
      for name,kb in kbs.items():
         kb.Enable(True)
   
   for i,(s,pp) in enumerate(plans):
      
      s()
      m.SendCommand('TagCurrentSubset {} setup{} true'.format(r.GetName(),i+1))
      
      p.InitPlan(r,pp)
      
      t = openravepy.RaveCreateTrajectory(e, '')
      tic = time.time()
      p.PlanPath(t)
      toc = time.time()
      times.append(toc - tic)
      trajs.append(t)


else: # without inter-step
   
   for i,(s,pp) in enumerate(plans):
      
      m = openravepy.RaveCreateModule(e, 'SubsetManager')
      e.Add(m, False, 'ssm')
      p = openravepy.RaveCreatePlanner(e, 'MultiSetPRM')
      p.SendCommand('UseSubsetManager ssm')
      p.SendCommand('SetRoadmap class=RoadmapSampledConst seed=419884521 batch_n=1000 radius=2')
      
      if w_selfcc:
         for name,kb in kbs.items():
            kb.Enable(False)
         m.SendCommand('TagCurrentSubset {} selfcc true'.format(r.GetName()))
         for name,kb in kbs.items():
            kb.Enable(True)
      
      s()
      m.SendCommand('TagCurrentSubset {} setup{} true'.format(r.GetName(),i+1))
      
      p.InitPlan(r,pp)
      
      if w_selfcc:
         p.SendCommand('CacheSetLocation mycache')
         p.SendCommand('CacheLoad 1')
      
      t = openravepy.RaveCreateTrajectory(e, '')
      tic = time.time()
      p.PlanPath(t)
      toc = time.time()
      times.append(toc - tic)
      trajs.append(t)
      
      e.Remove(m)
      
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
