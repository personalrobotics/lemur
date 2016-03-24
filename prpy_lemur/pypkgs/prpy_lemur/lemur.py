from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import division
import collections
import openravepy
import prpy.planning.base
import prpy_lemur.roadmaps

class LEMURPlanner(prpy.planning.base.BasePlanner):
   
   # params can be passed by name to constructor or planning methods
   LEMURParams = collections.namedtuple('LEMURParams',
      ['roadmap', 'check_cost', 'search_type', 'num_batches_init', 'max_batches'])
   
   def __init__(self, **kw_args):
      super(LEMURPlanner, self).__init__()
      
      self.planner = openravepy.RaveCreatePlanner(self.env, 'LEMUR')
      if self.planner is None:
         raise prpy.planning.base.UnsupportedPlanningError('Unable to create LEMUR planner.')
      
      P = LEMURPlanner.LEMURParams
      self.defaults = P(*[None for f in P._fields])._replace(**kw_args)
      
   def __str__(self):
      return 'LEMUR'
   
   @prpy.planning.base.PlanningMethod
   def PlanToConfiguration(self, robot, q_goal, **kw_args):
      
      params = self.defaults._replace(**kw_args)
      
      # serialize parameters
      orparams = openravepy.Planner.PlannerParameters()
      orparams.SetRobotActiveJoints(robot)
      orparams.SetGoalConfig(q_goal)
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
      orparams.SetExtraParameters('\n'.join(xml))
      self.planner.InitPlan(robot, orparams)
      
      # plan path
      traj = openravepy.RaveCreateTrajectory(self.env, '')
      status = self.planner.PlanPath(traj)
      if status == openravepy.PlannerStatus.HasSolution:
         return traj
      else:
         raise prpy.planning.base.PlanningError('LEMUR status: {}'.format(status))
