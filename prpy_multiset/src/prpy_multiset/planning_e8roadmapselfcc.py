from __future__ import print_function, unicode_literals, absolute_import, division
import openravepy
import prpy.planning.base

class E8RoadmapSelfCCPlanner(prpy.planning.base.BasePlanner):
   
   def __init__(self):
      super(E8RoadmapSelfCCPlanner, self).__init__()
      self.planner = openravepy.RaveCreatePlanner(self.env, 'E8RoadmapSelfCC')
      self.roadmap_id = 'RGG(n=1000 radius=2.0 seed=1)'
      
   def __str__(self):
      return 'E8RoadmapSelfCCPlanner'
   
   def GetSelfHash(self, robot):
      params = openravepy.Planner.PlannerParameters()
      params.SetRobotActiveJoints(robot)
      params.SetInitialConfig([])
      params.SetGoalConfig([])
      params.SetExtraParameters(''
         + '<roadmap_id>{}</roadmap_id>'.format(self.roadmap_id)
      )
      self.planner.InitPlan(robot, params)
      return self.planner.SendCommand('GetSelfHash')
   
   def CacheCalculateSave(self, robot):
      params = openravepy.Planner.PlannerParameters()
      params.SetRobotActiveJoints(robot)
      params.SetInitialConfig([])
      params.SetGoalConfig([])
      params.SetExtraParameters(''
         + '<roadmap_id>{}</roadmap_id>'.format(self.roadmap_id)
      )
      self.planner.InitPlan(robot, params)
      return self.planner.SendCommand('CacheCalculateSave')

   @prpy.planning.base.PlanningMethod
   def PlanToConfiguration(self, robot, q_goal, **kwargs):
      q_start = robot.GetActiveDOFValues()
      
      # initialize planner
      params = openravepy.Planner.PlannerParameters()
      params.SetRobotActiveJoints(robot)
      params.SetGoalConfig(q_goal)
      params.SetExtraParameters(''
         + '<roadmap_id>{}</roadmap_id>'.format(self.roadmap_id)
      )
      self.planner.InitPlan(robot, params)
      
      # plan path
      traj = openravepy.RaveCreateTrajectory(self.env, '')
      status = self.planner.PlanPath(traj)
      if status == openravepy.PlannerStatus.HasSolution:
         return traj
      else:
         raise prpy.planning.base.PlanningError(
            'planner status: {}'.format(status))
