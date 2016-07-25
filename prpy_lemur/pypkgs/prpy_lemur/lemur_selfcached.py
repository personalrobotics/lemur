from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import division
import collections
import errno
import os
import openravepy
import prpy.planning.base
import prpy_lemur.lemur
import prpy_lemur.roadmaps

class LEMURSelfCachedPlanner(prpy_lemur.lemur.LEMURPlanner):
   
   def __init__(self, **kw_args):
      super(LEMURSelfCachedPlanner, self).__init__(**kw_args)
      
   def __str__(self):
      return 'LEMURSelfCached'

   # this context manager adds a family module temporarily to the environment
   class AddedFamilyModule:
      def __init__(self, env, robot):
         self.env = env
         self.robot = robot
      def __enter__(self):
         self.family = openravepy.RaveCreateModule(self.env, 'Family')
         if self.family is None:
            raise prpy.planning.base.UnsupportedPlanningError('Unable to create Family module.')
         self.env.Add(self.family, False, '--robot-name={}'.format(self.robot.GetName()))
         return self.family
      def __exit__(self, exc_type, exc_value, traceback):
         self.env.Remove(self.family)
   
   @staticmethod
   def get_setcache_path(family, roadmap, read_only):
      
      # get family id
      family_id = family.SendCommand('GetFamilyId')
      
      # get roadmap id
      roadmap_id = prpy_lemur.roadmaps.get_roadmap_id(roadmap)
      
      setcache_filename = 'prpy_lemur/lemur_selfcached/family-{}-roadmap-{}.txt'.format(family_id,roadmap_id)
      setcache_path = openravepy.RaveFindDatabaseFile(setcache_filename, read_only) # bRead

      if setcache_path == '':
         if read_only:
            openravepy.raveLogWarn('Self cache database file not found:')
            openravepy.raveLogWarn('{}'.format(setcache_filename))
         else:
            raise RuntimeError('cant save to openrave database!')
      
      return setcache_path
   
   @prpy.planning.base.PlanningMethod
   def Generate(self, robot, num_batches, **kw_args):
      
      params = self.defaults._replace(max_batches=num_batches, **kw_args)
      
      # lock env, save robot state
      # and create family
      with robot, type(self).AddedFamilyModule(self.env, robot) as family:
      
         setcache_path = self.get_setcache_path(family, params.roadmap, read_only=False)
      
         # disable any link that moves due to non-active dofs
         openravepy.raveLogInfo('[Generate] Computing self-checked set cache for these links:')
         for linkindex,link in enumerate(robot.GetLinks()):
            inactive_affected = False
            for dofindex in range(robot.GetDOF()):
               if dofindex in robot.GetActiveDOFIndices():
                  continue
               joint = robot.GetJointFromDOFIndex(dofindex)
               jointindex = robot.GetJoints().index(joint)
               if robot.DoesAffect(jointindex, linkindex):
                  inactive_affected = True
            if inactive_affected:
               link.Enable(False)
            else:
               openravepy.raveLogInfo('[Generate]  [{}] {} ({} geoms)'.format(
                  linkindex, link.GetName(),
                  len(link.GetGeometries())))
         
         # save self-check
         family.SendCommand('Let Self = $live')

         openravepy.raveLogInfo('[Generate] Current family:')
         family.SendCommand('PrintCurrentFamily')

         self_header = family.SendCommand('GetHeaderFromSet Self')
         openravepy.raveLogInfo('[Generate] Self header:')
         for line in self_header.rstrip('\n').split('\n'):
            openravepy.raveLogInfo('[Generate] {}'.format(line))

         # create a planner
         planner = openravepy.RaveCreatePlanner(self.env, 'FamilyPlanner')
         if planner is None:
            raise prpy.planning.base.UnsupportedPlanningError('Unable to create FamilyPlanner planner.')

         # create params
         orparams = openravepy.Planner.PlannerParameters()
         orparams.SetRobotActiveJoints(robot)
         orparams.SetInitialConfig([])
         orparams.SetGoalConfig([])
         xml = self.xml_from_params(params)
         xml.append('<solve_all>true</solve_all>')
         xml.append('<family_module>{}</family_module>'.format(family.SendCommand('GetInstanceId')))
         xml.append('<family_setcaches><setcache><name>Self</name><filename>{}</filename></setcache></family_setcaches>'.format(setcache_path))
         orparams.SetExtraParameters('\n'.join(xml))

         success = planner.InitPlan(robot, orparams)
         if not success:
            raise RuntimeError('InitPlan failed!')

         result = planner.PlanPath(None)
         if result != openravepy.PlannerStatus.HasSolution:
            raise RuntimeError('Planning failed!')

         # ensure directories exist
         # from http://stackoverflow.com/a/5032238/5228520
         try:
            os.makedirs(os.path.dirname(setcache_path))
         except OSError as exception:
            if exception.errno != errno.EEXIST:
               raise

         openravepy.raveLogInfo('[Generate] Saving set cache ...')
         planner.SendCommand('SaveSetCaches')
         
         # return dummy trajectory
         traj = openravepy.RaveCreateTrajectory(self.env, '')
         traj.Init(robot.GetActiveConfigurationSpecification())
         return traj

   @prpy.planning.base.PlanningMethod
   def PlanToConfiguration(self, robot, goal_config, **kw_args):
      
      params = self.defaults._replace(**kw_args)
      
      # create family
      with type(self).AddedFamilyModule(self.env, robot) as family:

         setcache_path = self.get_setcache_path(family, params.roadmap, read_only=True)
      
         planner = openravepy.RaveCreatePlanner(self.env, 'FamilyPlanner')
         if planner is None:
            raise prpy.planning.base.UnsupportedPlanningError('Unable to create FamilyPlanner planner.')
         
         # serialize parameters
         orparams = openravepy.Planner.PlannerParameters()
         orparams.SetRobotActiveJoints(robot)
         orparams.SetGoalConfig(goal_config)
         xml = self.xml_from_params(params)
         xml.append('<family_module>{}</family_module>'.format(family.SendCommand('GetInstanceId')))
         if setcache_path != '':
            xml.append('<family_setcaches><setcache><filename>{}</filename></setcache></family_setcaches>'.format(setcache_path))
         orparams.SetExtraParameters('\n'.join(xml))
         planner.InitPlan(robot, orparams)
         
         # plan path
         traj = openravepy.RaveCreateTrajectory(self.env, '')
         status = planner.PlanPath(traj)
         if status == openravepy.PlannerStatus.HasSolution:
            return traj
         else:
            raise prpy.planning.base.PlanningError('LEMUR status: {}'.format(status))

   @prpy.planning.base.PlanningMethod
   def PlanToConfigurations(self, robot, goal_configs, **kw_args):
      
      params = self.defaults._replace(**kw_args)
      
      # create family
      with type(self).AddedFamilyModule(self.env, robot) as family:

         setcache_path = self.get_setcache_path(family, params.roadmap, read_only=True)
      
         planner = openravepy.RaveCreatePlanner(self.env, 'FamilyPlanner')
         if planner is None:
            raise prpy.planning.base.UnsupportedPlanningError('Unable to create FamilyPlanner planner.')
         
         # serialize parameters
         orparams = openravepy.Planner.PlannerParameters()
         orparams.SetRobotActiveJoints(robot)
         goal_configs_stacked = []
         for goal_config in goal_configs:
            goal_configs_stacked.extend(goal_config)
         orparams.SetGoalConfig(goal_configs_stacked)
         xml = self.xml_from_params(params)
         xml.append('<family_module>{}</family_module>'.format(family.SendCommand('GetInstanceId')))
         if setcache_path != '':
            xml.append('<family_setcaches><setcache><filename>{}</filename></setcache></family_setcaches>'.format(setcache_path))
         orparams.SetExtraParameters('\n'.join(xml))
         planner.InitPlan(robot, orparams)
         
         # plan path
         traj = openravepy.RaveCreateTrajectory(self.env, '')
         status = planner.PlanPath(traj)
         if status == openravepy.PlannerStatus.HasSolution:
            return traj
         else:
            raise prpy.planning.base.PlanningError('LEMUR status: {}'.format(status))
