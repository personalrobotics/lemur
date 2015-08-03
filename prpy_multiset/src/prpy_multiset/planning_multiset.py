import openravepy
import prpy.planning.base

# for now, this instantiates a new planner object for each call!
class MultisetPlanner(prpy.planning.base.BasePlanner):
   
   def __init__(self):
      super(MultisetPlanner, self).__init__()
      
   def __str__(self):
      return 'MultisetPlanner'

   @prpy.planning.base.PlanningMethod
   def PlanToConfiguration(self, robot, q_goal, **kwargs):
      print('inner env:', self.env)
      
      m = None
      t = None
      
      q_start = robot.GetActiveDOFValues()
      
      try:
         m = openravepy.RaveCreateModule(self.env, 'SubsetManager')
         self.env.Add(m, False, 'ssm')
         p = openravepy.RaveCreatePlanner(self.env, 'MultiSetPRM')
         p.SendCommand('UseSubsetManager ssm')
         p.SendCommand('SetRoadmap class=RoadmapSampledConst seed=419884521 batch_n=1000 radius=2')

         pp = openravepy.Planner.PlannerParameters()
         pp.SetRobotActiveJoints(robot)
         pp.SetGoalConfig(q_goal)
         pp.SetExtraParameters(''
            + '<lambda>{}</lambda>'.format(0.5)
            + '<interroot_radius>2.0</interroot_radius>'
         )
         
         p.InitPlan(robot,pp)
         
         t = openravepy.RaveCreateTrajectory(self.env, '')
         ret = p.PlanPath(t)
         print('returned:', ret)
      
      finally:
         if m is not None:
            self.env.Remove(m)
      
      return t
      
      #raise prpy.planning.base.PlanningError(
      #   'planning not yet implemented!')
