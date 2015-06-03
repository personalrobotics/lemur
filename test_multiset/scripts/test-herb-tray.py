#!/usr/bin/env python2

# File: test-herb-tray.py
# Author: Chris Dellin <cdellin@gmail.com>
# Copyright: 2015 Carnegie Mellon University
# License: BSD

from __future__ import print_function, unicode_literals, absolute_import, division

import argparse
import atexit
import sys
import time
import xml.etree.ElementTree
import numpy
import openravepy

def set_camera(orviewer, orcamparams):
   root = xml.etree.ElementTree.fromstring('<root>{}</root>'.format(orcamparams))
   ra = map(float,root.find('camrotationaxis').text.split())
   axisangle = numpy.array(ra[0:3]) * ra[3] * numpy.pi / 180.0
   H = openravepy.matrixFromAxisAngle(axisangle)
   H[0:3,3] = map(float,root.find('camtrans').text.split())
   e.GetViewer().SetCamera(H, float(root.find('camfocal').text))

class FridgeProblem:
   def __init__(self, env):
      self.robot = None
      self.setups = [] # N setup functions
      self.roots = [] # N+1 sets of roots
      
      # ideas:
      #
      #especially in clutter! (but no narrow passages)
      #
      #3 objects
      #
      #some iks get you stuck
      # - i can provoke this with iks i think
      # - do i need a more confined space for this?
      # - JOINT LIMITS!
      #
      #some grasps dont let you use / place the object
      # - pick up a glass too low, cant place in try cause of lip!
      #
      #some placements preclude subsequent motion / placements
      # - place it so you cant escape!
      # - this can be up against other objects!

      # hardcoded poses (openrave order, qw qx qy qz x y z)
      S12 = numpy.sqrt(0.5)
      T = lambda *pose: openravepy.matrixFromPose(pose)
      self.T_r             = T( 0.93268, 0.,  0., -0.36071, 0.518, 0.777, 0.       )
      self.T_tray          = T( 1.,      0.,  0.,  0.,      1.51,  1.15,  0.9165   )
      self.T_pitcher_f     = T( 0.97131, 0.,  0.,  0.23780, 1.346, 0.400, 0.76    )
      #self.T_pitcher_t     = T( 0.97131, 0.,  0.,  0.23780, 1.561, 1.330, 0.935    )
      self.T_pitcher_t     = T( 0.88072, 0.,  0., -0.47364, 1.49,  0.989, 0.94    )
      self.T_pitcher_grasp = T( 0.5,     0.5, 0.5, 0.5,    -0.18,  0.,    0.15     )
      self.T_bottle_f      = T( numpy.cos(-0.00),      0.,  0.,  numpy.sin(-0.00),      1.36,  0.42,  0.31     )
      self.T_bottle_t      = T( 1.,      0.,  0.,  0.,      1.52,  1.28,  0.94    )
      self.T_bottle_grasp  = T( 0.5,     0.5, 0.5, 0.5,    -0.13,  0.,    0.15     )
      self.T_beer          = T( S12,     S12, 0.,  0.,      1.45,  0.49,  0.75     )
      self.T_ee_palm       = T( 1.,      0.,  0.,  0.,      0.,    0.,    0.151072 ) # 0.1365
      
      # load a robot, ik solver
      m = openravepy.RaveCreateModule(env, 'urdf')
      r_name = m.SendCommand('load devel/share/herb_description/robots/herb.urdf devel/share/herb_description/robots/herb.srdf')
      self.robot = env.GetRobot(r_name)
      self.robot.SetTransform(self.T_r)

      # the proper joint order! haha
      joint_order = [
         '/right/j1',
         '/right/j2',
         '/right/j3',
         '/right/j4',
         '/right/j5',
         '/right/j6',
         '/right/j7',
         '/right/j01',
         '/right/j11',
         '/right/j21',
         '/right/j00',
         '/left/j1',
         '/left/j2',
         '/left/j3',
         '/left/j4',
         '/left/j5',
         '/left/j6',
         '/left/j7',
         '/left/j01',
         '/left/j11',
         '/left/j21',
         '/left/j00'
      ]
      self.di = [self.robot.GetJoint(j).GetDOFIndex() for j in joint_order]

      # robot dof values
      r_dofvals = [
         1.5*numpy.pi, -1.75, 0.0, 1.9, 0., 0.53, 0., # right arm
         1.15, 1.15, 1.15, 0., # right hand
         0.630, -1.900,  0.3, 1.9, 0.6, 0.5, 0., # left arm
         #2.3,2.3,2.3,0. # left hand
         #0.95, 0.95, 0.95, 0
         0.7, 0.7, 0.7, 0
      ]
      self.robot.SetDOFValues(r_dofvals, self.di)
      self.robot.SetActiveManipulator('left')
      self.robot.SetActiveDOFs(self.robot.GetActiveManipulator().GetArmIndices())
      ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,
         iktype=openravepy.IkParameterization.Type.Transform6D)
      if not ikmodel.load():
         ikmodel.autogenerate()

      self.kbs = {}
      self.kbs['kitchen'] = env.ReadKinBodyXMLFile('environments/pr_kitchen_fridgedoor.kinbody.xml')
      self.kbs['tray'] = env.ReadKinBodyXMLFile('objects/wicker_tray.kinbody.xml')
      self.kbs['pitcher'] = env.ReadKinBodyXMLFile('objects/household/teakettle.kinbody.xml')
      self.kbs['bottle'] = env.ReadKinBodyXMLFile('objects/household/bottle.kinbody.xml')
      #self.kbs['beer'] = env.ReadKinBodyXMLFile('objects/household/beer.kinbody.xml')
      for name,kb in self.kbs.items():
         if not kb:
            raise RuntimeError('kinbody {} not found!'.format(name))
         env.Add(kb)
      # open fridge door
      self.kbs['kitchen'].SetDOFValues([1.658062],
         [self.kbs['kitchen'].GetJoint('fridge_door').GetDOFIndex()])
      # place objects
      self.kbs['tray'].SetTransform(self.T_tray)
      self.kbs['pitcher'].SetTransform(self.T_pitcher_f)
      self.kbs['bottle'].SetTransform(self.T_bottle_f)
      #self.kbs['beer'].SetTransform(self.T_beer)
      
      # save setups
      self.setups = [
         self.setup1,
         self.setup2,
         self.setup3,
         self.setup4,
         self.setup5
      ]
      
      # save roots
      self.roots.append([[0.630, -1.900,  0.3, 1.9, 0.6, 0.5, 0.]])
      
      # pitcher grasp configs
      self.setup1()
      ikgoals = self.robot.GetActiveManipulator().FindIKSolutions(
         reduce(numpy.dot,
            [self.kbs['pitcher'].GetTransform(),
            self.T_pitcher_grasp,
            numpy.linalg.inv(self.T_ee_palm)]),
         openravepy.IkFilterOptions.CheckEnvCollisions)
      self.roots.append(ikgoals)
      
      # pitcher release configs
      self.setup3()
      ikgoals = self.robot.GetActiveManipulator().FindIKSolutions(
         reduce(numpy.dot,
            [self.kbs['pitcher'].GetTransform(),
            self.T_pitcher_grasp,
            numpy.linalg.inv(self.T_ee_palm)]),
         openravepy.IkFilterOptions.CheckEnvCollisions)
      self.roots.append(ikgoals)
      
      # bottle grasp configs
      ikgoals = self.robot.GetActiveManipulator().FindIKSolutions(
         reduce(numpy.dot,
            [self.kbs['bottle'].GetTransform(),
            self.T_bottle_grasp,
            numpy.linalg.inv(self.T_ee_palm)]),
         openravepy.IkFilterOptions.CheckEnvCollisions)
      self.roots.append(ikgoals)
      
      # bottle release configs
      self.setup5()
      ikgoals = self.robot.GetActiveManipulator().FindIKSolutions(
         reduce(numpy.dot,
            [self.kbs['bottle'].GetTransform(),
            self.T_bottle_grasp,
            numpy.linalg.inv(self.T_ee_palm)]),
         openravepy.IkFilterOptions.CheckEnvCollisions)
      self.roots.append(ikgoals)
      
      # return back where we started
      self.roots.append(self.roots[0])

   # pitcher is in fridge
   def setup1(self):
      self.robot.ReleaseAllGrabbed()
      self.kbs['pitcher'].SetTransform(self.T_pitcher_f)
      self.kbs['bottle'].SetTransform(self.T_bottle_f)
   
   # pitcher is grabbed by robot
   def setup2(self):
      self.robot.ReleaseAllGrabbed()
      self.kbs['bottle'].SetTransform(self.T_bottle_f)
      self.kbs['pitcher'].SetTransform(reduce(numpy.dot,
         [self.robot.GetActiveManipulator().GetEndEffectorTransform(),
         self.T_ee_palm,
         numpy.linalg.inv(self.T_pitcher_grasp)]
      ))
      self.robot.Grab(self.kbs['pitcher'])

   # pitcher is on the tray, bottle still in fridge
   def setup3(self):
      self.robot.ReleaseAllGrabbed()
      self.kbs['pitcher'].SetTransform(self.T_pitcher_t)
      self.kbs['bottle'].SetTransform(self.T_bottle_f)
   
   # bottle is grabbed by robot
   def setup4(self):
      self.robot.ReleaseAllGrabbed()
      self.kbs['pitcher'].SetTransform(self.T_pitcher_t)
      self.kbs['bottle'].SetTransform(reduce(numpy.dot,
         [self.robot.GetActiveManipulator().GetEndEffectorTransform(),
         self.T_ee_palm,
         numpy.linalg.inv(self.T_bottle_grasp)]
      ))
      self.robot.Grab(self.kbs['bottle'])

   # bottle is on the tray
   def setup5(self):
      self.robot.ReleaseAllGrabbed()
      self.kbs['pitcher'].SetTransform(self.T_pitcher_t)
      self.kbs['bottle'].SetTransform(self.T_bottle_t)



parser = argparse.ArgumentParser(description='HERB problem experiment script.')
parser.add_argument('--planner')
parser.add_argument('--seed', type=int)
parser.add_argument('--view-problem', default=False, action='store_true')
parser.add_argument('--timelimit',type=float,default=600.0)
parser.add_argument('--outfile')
parser.add_argument('--outfile-lineprefix')
parser.add_argument('--stop-after', type=int) # do only these steps
parser.add_argument('--w-viewer', default=False, action='store_true')
parser.add_argument('--w-any-to-any', default=False, action='store_true')

# for planner=MultiSetPRM
parser.add_argument('--lambda', dest='lambda_', type=float)
#parser.add_argument('--radius', type=float)
#parser.add_argument('--gamma-rel', type=float)
parser.add_argument('--w-interstep', default=False, action='store_true')
parser.add_argument('--w-selfcc', default=False, action='store_true')
parser.add_argument('--cache-up-to', type=int)

# for planner=OMPL_RRTConnect
parser.add_argument('--range', type=float)

args = parser.parse_args()

# create an environment, load the robot (wam)
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
e = openravepy.Environment()
atexit.register(e.Destroy)

e.GetCollisionChecker().SetCollisionOptions(
   openravepy.CollisionOptions.ActiveDOFs)

if args.w_viewer:
   e.SetViewer('qtcoin')
   set_camera(e.GetViewer, '''
      <camtrans>-0.639213 -0.635469 2.295384</camtrans>
      <camrotationaxis>-0.880159 0.435158 -0.189624 132.419475</camrotationaxis>
      <camfocal>2.286811</camfocal>''')
   e.GetViewer().SetSize(1500,560)
   # gimp cropping: 1440 485 -30 -23

# create the problem (with the robot)
prob = FridgeProblem(e)


if args.view_problem:
   
   # simple view
   #prob.setup1()
   
   prob.setup1()
   T_pitcher_fridge = prob.kbs['pitcher'].GetTransform()
      
   # for introduction (example problem)
   while True:
      print('viewing problem ...')
      prob.setup1()
      prob.robot.SetActiveDOFValues(prob.roots[0][0])
      raw_input('initial config ...')
      
      # show a bottle grasp
      for i in range(len(prob.roots[3])):
         prob.robot.SetActiveDOFValues(prob.roots[3][i])
         raw_input('bottle grasp ...')
      
      prob.setup5()
      prob.kbs['pitcher'].SetTransform(T_pitcher_fridge)
      prob.robot.SetActiveDOFValues(prob.roots[4][0])
      
      raw_input('enter to restart ...')
   
   # for multiset examples
   while True:
      
      prob.kbs['kitchen'].SetTransform([1,0,0,0,0,0,0])
      
      prob.robot.SetDOFValues([1.0,1.0,1.0,0], prob.di[18:22])
      q = [2.1, -1.9, 0., 1.5, -0.4, 0.4, 3.0]
      
      prob.setup5()
      prob.kbs['pitcher'].SetTransform(T_pitcher_fridge)
      prob.robot.SetActiveDOFValues(q)
      raw_input('one')
      
      prob.kbs['bottle'].SetVisible(False)
      raw_input('two')
      prob.kbs['bottle'].SetVisible(True)
      
      prob.setup4()
      prob.kbs['pitcher'].SetTransform(T_pitcher_fridge)
      prob.robot.SetActiveDOFValues(q)
      raw_input('three')
      
      for kb in prob.kbs.values():
         kb.SetVisible(False)
         kb.Enable(False)
      raw_input('four')
      
      # workcell decomp
      k = openravepy.RaveCreateKinBody(e,'')
      k.SetName('blah')
      aabbs = numpy.zeros((0,6))
      aabb = numpy.array([[0,0,0, 0.2,0.2,0.2]])
      aabbs = numpy.vstack((aabbs,aabb))
      k.InitFromBoxes(aabbs,True)
      e.Add(k)
      k.SetTransform([1,0,0,0, 1.2,1.2,0.6])
      raw_input('five')
      T = prob.kbs['bottle'].GetTransform()
      T = numpy.dot(T,numpy.array(
         [[1,0,0,0],[0,1,0,0],[0,0,1,0.15],[0,0,0,1]]))
      k.SetTransform(T)
      raw_input('six')
      e.Remove(k)
      
      for kb in prob.kbs.values():
         kb.SetVisible(True)
         kb.Enable(True)
      
      
         
   raw_input('enter to quit! ...')
   
   print(prob.robot.GetActiveDOFValues())
   
   exit()


# check planners
planners = ['MultiSetPRM', 'OMPL_RRTConnect', 'OMPL_LazyPRM', 'OMPL_LBKPIECE1']
if args.planner not in planners:
   raise RuntimeError('planner must be in:', planners)
if args.planner == 'MultiSetPRM':
   if None in [args.lambda_,args.seed]:
      raise RuntimeError('MultiSetPRM lamba parameter must be passed!')
if args.planner == 'OMPL_RRTConnect':
   if None in [args.range]:#, args.seed]:
      raise RuntimeError('OMPL_RRTConnect range parameter must be passed!')
if args.planner == 'OMPL_LazyPRM':
   if args.range is None:
      raise RuntimeError('OMPL_LazyPRM range parameter must be passed!')
if args.planner == 'OMPL_LBKPIECE1':
   if args.range is None:
      raise RuntimeError('OMPL_LBKPIECE1 range parameter must be passed!')


# do some sweet stuff
if args.planner == 'MultiSetPRM' and args.cache_up_to is not None:
   for kb in e.GetBodies():
      if kb.GetName() == prob.robot.GetName():
         continue
      kb.Enable(False)
   m = openravepy.RaveCreateModule(e, 'SubsetManager')
   e.Add(m, False, 'ssm')
   m.SendCommand('TagCurrentSubset {} selfcc true'.format(prob.robot.GetName()))
   m.SendCommand('DumpSubsets {} -'.format(prob.robot.GetName()))
   p = openravepy.RaveCreatePlanner(e, 'MultiSetPRM')
   p.SendCommand('SetRoadmap class=RoadmapSampledDensified seed={} batch_n=1000 gamma_rel=0.7'.format(args.seed)) # this one!
   p.SendCommand('UseSubsetManager ssm')
   pp_self = openravepy.Planner.PlannerParameters()
   pp_self.SetExtraParameters('<eval_subgraphs>{}</eval_subgraphs>'.format(args.cache_up_to))
   p.InitPlan(prob.robot, pp_self)
   t = openravepy.RaveCreateTrajectory(e, '')
   p.PlanPath(t)
   p.SendCommand('CacheSetLocation mycache-fridge')
   p.SendCommand('CacheSave')
   print('bailing super early!')
   exit()




for i,roots in enumerate(prob.roots):
   print('rootset {} has {} roots:'.format(i, len(roots)))
   for root in roots:
      print('  root: {}'.format(root))

if False:
   prob.setup5()
   #for root in prob.roots[1]:
   #   prob.robot.SetActiveDOFValues(root)
   #   raw_input('start')
   for root in prob.roots[4]:
      prob.robot.SetActiveDOFValues(root)
      raw_input('bottle_grasp')
   raw_input('enter to quit')
   exit()





#steps = [step1, step2, step3]
#if args.stop_after is not None:
#   steps = steps[0:args.stop_after]

setups = []
times = []
trajs = []

# create the nominal single planner instance
p = openravepy.RaveCreatePlanner(e, args.planner)

for i,setup in enumerate(prob.setups):
   
   if args.stop_after is not None and i == args.stop_after:
      break

   # set up for this step
   setup()
   
   setups.append(setup)
   
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
         #p.SendCommand('SetRoadmap class=RoadmapSampledConst seed={} batch_n=1000 radius=2'.format(args.seed))
         #p.SendCommand('SetRoadmap class=RoadmapSampledConst seed={} batch_n=1000 radius={}'.format(args.seed, args.radius))
         #p.SendCommand('SetRoadmap class=RoadmapSampledDensified seed={} batch_n=500 gamma_rel={}'.format(args.seed,args.gamma_rel))
         p.SendCommand('SetRoadmap class=RoadmapSampledDensified seed={} batch_n=1000 gamma_rel=0.7'.format(args.seed)) # this one!
         p.SendCommand('UseSubsetManager ssm')
         
         if args.w_selfcc:
            for name,kb in kbs.items():
               kb.Enable(False)
            m.SendCommand('TagCurrentSubset {} selfcc true'.format(prob.robot.GetName()))
            for name,kb in kbs.items():
               kb.Enable(True)

      m.SendCommand('TagCurrentSubset {} step{} true'.format(prob.robot.GetName(),i+1))
   
   # set up planner parameters
   pp = openravepy.Planner.PlannerParameters()
   pp.SetRobotActiveJoints(prob.robot)
   
   # starts
   vinitialconfig = []
   if args.w_any_to_any:
      for start in prob.roots[i]:
         vinitialconfig.extend(start)
   else:
      if trajs:
         vinitialconfig = trajs[-1].GetWaypoint(trajs[-1].GetNumWaypoints()-1)
      else:
         vinitialconfig = [0.630, -1.900,  0.3, 1.9, 0.6, 0.5, 0.]
   pp.SetInitialConfig(vinitialconfig)
   
   # goals
   vgoalconfig = []
   for goal in prob.roots[i+1]:
      vgoalconfig.extend(goal)
   pp.SetGoalConfig(vgoalconfig)
   
   # custom parameters, by planner
   if args.planner == 'MultiSetPRM':
      params = ''
      params += '<timelimit>{}</timelimit>'.format(args.timelimit)
      params += '<lambda>{}</lambda>'.format(args.lambda_)
      params += '<interroot_radius>2.0</interroot_radius>'
      pp.SetExtraParameters(params)
   elif args.planner in ['OMPL_RRTConnect','OMPL_LazyPRM','OMPL_LBKPIECE1']:
      params = ''
      params += '<time_limit>{}</time_limit>'.format(args.timelimit)
      params += '<range>{}</range>'.format(args.range)
      if i==0:
         params += '<seed>{}</seed>'.format(args.seed)
      pp.SetExtraParameters(params)
   else:
      raise RuntimeError('unknown planner!')
   
   p.InitPlan(prob.robot,pp)
   
   if args.planner == 'MultiSetPRM' and args.w_selfcc:
      p.SendCommand('CacheSetLocation mycache-fridge')
      p.SendCommand('CacheLoad 1')
   
   if args.planner == 'OMPL_RRTConnect' and args.seed is not None:
      p.SendCommand('SetSamplerSeed {}'.format(args.seed))
   
   t = openravepy.RaveCreateTrajectory(e, '')
   tic = time.time()
   p.PlanPath(t)
   toc = time.time()
   times.append(toc - tic)
   trajs.append(t)
   
   #q_last = t.GetWaypoint(t.GetNumWaypoints()-1)
   #print('setting config to:', map(float,q_last))
   #r.SetActiveDOFValues(q_last)

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
   
if e.GetViewer():
   for traj in trajs:
      openravepy.planningutils.RetimeActiveDOFTrajectory(traj, prob.robot)

   while True:
   
      for i,traj in enumerate(trajs):
         #raw_input('press enter to view trajectory {} ...'.format(i+1))
         setups[i]()
         
         if i == 0:
            raw_input('enter to view all trajs ...')
         
         # HACK: SET HAND SHAPE, ONLY FOR DISPLAY
         if i in [0,2,4]:
            prob.robot.SetDOFValues([0.7,0.7,0.7,0], prob.di[18:22])
         elif i == 1:
            prob.robot.SetDOFValues([1.3,1.3,1.3,0], prob.di[18:22])
         else:
            prob.robot.SetDOFValues([1.0,1.0,1.0,0], prob.di[18:22])
         
         prob.robot.GetController().SetPath(traj)
         while not prob.robot.GetController().IsDone():
            time.sleep(0.1)
         time.sleep(1.0)


exit()

raw_input('enter to quit!')
