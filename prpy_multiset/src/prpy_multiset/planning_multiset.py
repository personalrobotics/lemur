from __future__ import print_function, unicode_literals, absolute_import, division
import openravepy
import prpy.planning.base

# for now, this instantiates a new planner object for each call!
class MultisetPlanner(prpy.planning.base.BasePlanner):
   
   def __init__(self):
      super(MultisetPlanner, self).__init__()
      self.lambda_ = 0.5
      self.seed = 419884521
      
   def __str__(self):
      return 'MultisetPlanner'
   
   def set_lambda(self, lambda_):
      self.lambda_ = lambda_

   @prpy.planning.base.PlanningMethod
   def PlanToConfiguration(self, robot, q_goal, **kwargs):
      q_start = robot.GetActiveDOFValues()
      
      m = None
      try:
         # construct modules
         m = openravepy.RaveCreateModule(self.env, 'SubsetManager')
         self.env.Add(m, False, 'ssm')
         p = openravepy.RaveCreatePlanner(self.env, 'MultiSetPRM')
         p.SendCommand('UseSubsetManager ssm')
         p.SendCommand('SetRoadmap class=RoadmapSampledConst seed={} batch_n=1000 radius=2'.format(self.seed))

         # initialize planner
         pp = openravepy.Planner.PlannerParameters()
         pp.SetRobotActiveJoints(robot)
         pp.SetGoalConfig(q_goal)
         pp.SetExtraParameters(''
            + '<lambda>{}</lambda>'.format(self.lambda_)
            + '<interroot_radius>2.0</interroot_radius>'
         )
         p.InitPlan(robot,pp)
         
         # plan
         traj = openravepy.RaveCreateTrajectory(self.env, '')
         status = p.PlanPath(traj)
         if status == openravepy.PlannerStatus.HasSolution:
            return traj
         else:
            raise prpy.planning.base.PlanningError(
               'planner status: {}'.format(status))
      
      finally:
         if m is not None:
            self.env.Remove(m)
