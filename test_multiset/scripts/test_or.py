#!/usr/bin/env python2
from __future__ import print_function, unicode_literals, absolute_import, division

import atexit
import math
import sys
import time
import numpy
import openravepy
import libcd

#lambda_ = 0.0001
#lambda_ = 0.5
lambda_ = 0.9999
w_inter_step = True
w_selfcc = True

def multidot(*args):
   res = args[0]
   for m in args[1:]:
      res = numpy.dot(res,m)
   return res

def H_from_pose(pose):
   H = numpy.eye(4)
   libcd.kin.pose_to_H(pose,H,True)
   return H

M_SQRT1_2 = math.sqrt(0.5)

# hardcoded poses
pose_ee_palm = [ 0., 0., 0.1365, 0., 0., 0., 1. ]
pose_mug_grasp1 = [ 0.15, 0., 0.09, -0.5, -0.5, 0.5, 0.5 ]
pose_mugT = [ -0.3975, 1.61, 0.735, 0., 0., 1., 0. ] # table
pose_mugD = [ -1.1, 2.3, 0.0, 0.,0.,M_SQRT1_2,-M_SQRT1_2 ] # bin
pose_mug_drop = [ -1.1, 2.3, 0.735, 0.,0.,M_SQRT1_2,-M_SQRT1_2 ] # drop location

# create an environment, load the robot (wam)
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
e = openravepy.Environment()
atexit.register(e.Destroy)

#e.SetViewer('qtcoin')

# load a robot, ik solver
r = e.ReadRobotXMLFile('robots/herb2_padded_nosensors.robot.xml')
e.Add(r)
r.SetTransform(H_from_pose([-0.3975,2.38,0., 0.,0.,-M_SQRT1_2,M_SQRT1_2]));
dofvals = [
   5.759, -1.972, -0.22, 1.9, 0., 0., 0., 1.3,1.3,1.3,0., # right
   0.630, -1.900,  0.15, 1.9, 0., 0., 0., 2.3,2.3,2.3,0.  # left
]
r.SetDOFValues(dofvals,range(len(dofvals)))
r.SetActiveManipulator('right_wam')
r.SetActiveDOFs(r.GetActiveManipulator().GetArmIndices())
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(r,
   iktype=openravepy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
   ikmodel.autogenerate()

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
      raise RuntimeError('{} not found!'.format(name))
   e.Add(kb)
kbs['table'].SetTransform(H_from_pose([-0.3975,1.61,0., 0.,0.,-M_SQRT1_2,M_SQRT1_2]))
kbs['bin'].SetTransform(H_from_pose([-1.1,2.3,0.0, 0.,0.,-M_SQRT1_2,M_SQRT1_2]))

# get iks for mug on table
kbs['mug'].SetTransform(H_from_pose(pose_mugT))
H = numpy.dot(
   numpy.dot(kbs['mug'].GetTransform(), H_from_pose(pose_mug_grasp1)),
   numpy.linalg.inv(H_from_pose(pose_ee_palm))
   )
mugiksT = r.GetActiveManipulator().FindIKSolutions(H, openravepy.IkFilterOptions.CheckEnvCollisions)

# get iks for mug at drop location
kbs['mug'].SetTransform(H_from_pose(pose_mug_drop))
H = numpy.dot(
   numpy.dot(kbs['mug'].GetTransform(), H_from_pose(pose_mug_grasp1)),
   numpy.linalg.inv(H_from_pose(pose_ee_palm))
   )
mugiksdrop = r.GetActiveManipulator().FindIKSolutions(H, openravepy.IkFilterOptions.CheckEnvCollisions)


# create the three problem definitions (encoded as planner parameters xmls)

def s1():
   r.Release(kbs['mug'])
   kbs['mug'].SetTransform(H_from_pose(pose_mugT))
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
   kbs['mug'].SetTransform(multidot(
      r.GetActiveManipulator().GetEndEffectorTransform(),
      H_from_pose(pose_ee_palm),
      numpy.linalg.inv(H_from_pose(pose_mug_grasp1))
      ))
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
   kbs['mug'].SetTransform(H_from_pose(pose_mugD))
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
      
      s()
      m.SendCommand('TagCurrentSubset {} setup{} true'.format(r.GetName(),i+1))
      
      if w_selfcc:
         for name,kb in kbs.items():
            kb.Enable(False)
         m.SendCommand('TagCurrentSubset {} selfcc true'.format(r.GetName()))
         for name,kb in kbs.items():
            kb.Enable(True)
      
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
      del m
      
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
