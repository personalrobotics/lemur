from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import division
import collections
import openravepy
import prpy.planning.base

# a PrPy FamilyPlanner is inherently stateful.
# it maintains a family module and a family planner in its environment
# (this is loaded on a per-robot basis)
class FamilyPlanner(prpy.planning.base.BasePlanner):

   # params can be passed by name to constructor or planning methods
   Params = collections.namedtuple('FamilyPlannerParams',
      ['roadmap', 'check_cost', 'search_type', 'eval_type', 'num_batches_init', 'max_batches'])

   def __init__(self, **kw_args):
      super(FamilyPlanner, self).__init__()

      # state
      # module is NOT added to environment between calls!
      self.module = None
      self.planner = None

      Params = type(self).Params
      self.defaults = Params(*[None for f in Params._fields])._replace(**kw_args)

   def __str__(self):
      return 'FamilyPlanner'

   @staticmethod
   def xml_from_params(params):
      xml = list()
      for k,v in params._asdict().items():
         if v is None:
            continue
         if k == 'roadmap':
            xml.append('<roadmap_type>{}</roadmap_type>'.format(type(v).__name__))
            for k,v in v._asdict().items():
               xml.append('<roadmap_param>{k}={v}</roadmap_param>'.format(k=k,v=v))
         else:
            xml.append('<{k}>{v}</{k}>'.format(k=k,v=v))
      return xml

   # should be idempotent
   # if this returns successfully, self.module and self.planner will exist
   def initialize(self, robot):

      # construct a new module/planner if necessary
      if self.module is None:
         self.module = openravepy.RaveCreateModule(self.env, 'Family')
         if self.module is None:
            raise prpy.planning.base.UnsupportedPlanningError('Unable to create Family module.')

      if self.planner is None:
         self.planner = openravepy.RaveCreatePlanner(self.env, 'FamilyPlanner')
         if self.planner is None:
            raise prpy.planning.base.UnsupportedPlanningError('Unable to create FamilyPlanner planner.')

    # this context manager adds the planner's family module temporarily to the environment
   class AddedFamilyModule:
      def __init__(self, planner, robot):
         self.planner = planner
         self.robot = robot
      def __enter__(self):
         self.planner.env.Add(self.planner.module, False, '--robot-name={}'.format(self.robot.GetName()))
      def __exit__(self, exc_type, exc_value, traceback):
         self.planner.env.Remove(self.planner.module)

   @prpy.planning.base.PlanningMethod
   def TagCurrentSet(self, robot, setname):

      self.initialize(robot)

      with self.AddedFamilyModule(self,robot):
         self.module.SendCommand('Let {} = $live'.format(setname))

      traj = openravepy.RaveCreateTrajectory(self.env, '')
      traj.Init(robot.GetActiveConfigurationSpecification())
      return traj

   @prpy.planning.base.PlanningMethod
   def PlanToConfiguration(self, robot, goal_config, **kw_args):

      self.initialize(robot)

      # get parameters
      params = self.defaults._replace(**kw_args)

      # serialize parameters
      orparams = openravepy.Planner.PlannerParameters()
      orparams.SetRobotActiveJoints(robot)
      orparams.SetGoalConfig(goal_config)
      xml = self.xml_from_params(params)
      xml.append('<family_module>{}</family_module>'.format(self.module.SendCommand('GetInstanceId')))
      orparams.SetExtraParameters('\n'.join(xml))

      with self.AddedFamilyModule(self,robot):

         # initialize planner
         self.planner.InitPlan(robot, orparams)

         # plan path
         traj = openravepy.RaveCreateTrajectory(self.env, '')
         status = self.planner.PlanPath(traj)

      if status == openravepy.PlannerStatus.HasSolution:
         return traj
      else:
         raise prpy.planning.base.PlanningError('FamilyPlanner status: {}'.format(status))
