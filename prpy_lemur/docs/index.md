prpy_lemur: LEMUR for PrPy {#index}
==========================

[prpy][github-prpy] bindings for Chris Dellin's LEMUR planner.  For
now, only a few features are supported.  For example, caching between
calls (within a single planner instance, or through cached files) is
not yet supported.

LEMUR Planner Usage
-------------------

The core planner abides by the standard [prpy][github-prpy] planning
interface.

    # Plan for the barrettwam robot arm
    import openravepy
    env = openravepy.Environment()
    robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
    env.Add(robot)
    robot.SetActiveDOFs(range(7))
    
    # Create a LEMUR prpy planner instance
    import prpy_lemur
    planner = prpy_lemur.LEMURPlanner(
       roadmap=prpy_lemur.roadmaps.Halton(num=1000, radius=2.0))
    
    # Invoke a planning method
    q_start = [-2.0,-0.5,-0.2,-0.5, 0.0, 0.0, 0.0 ]
    q_goal  = [ 2.0, 0.5, 0.2, 0.5, 0.0, 0.0, 0.0 ]
    robot.SetActiveDOFValues(q_start)  
    traj = planner.PlanToConfiguration(robot, q_goal, max_batches=1)

Planning Methods Supported
--------------------------

These [prpy][github-prpy] planning methods are currently supported:

- `PlanToConfiguration(robot, goal_config)`
- `PlanToConfigurations(robot, goal_configs)`

Self-Collision Checked Planner
------------------------------

`prpy_lemur` also includes experimental bindings for a family planner
which is able to cache self-collision checks.  First, construct a
planner of the correct type:

    planner = prpy_lemur.LEMURSelfCachedPlanner(
       roadmap=prpy_lemur.roadmaps.Halton(num=1000, radius=2.0))

Then, you can perform self-collision checks:

    planner.Generate(robot, num_batches=1)

Future uses of this planner type should now rely on the cached
collision checks for their plans.

License
-------

LEMUR is written by Chris Dellin at the [Personal Robotics Lab][prlab]
at the [Robotics Intitute][ri], [Carnegie Mellon University][cmu].  It
was released on April 9, 2015 under a BSD license.

Source code for this package is on [GitHub][github-prpy-lemur].

[cmu]: http://www.cmu.edu/
[github-prpy]: https://github.com/personalrobotics/prpy/
[github-prpy-lemur]: https://github.com/personalrobotics/lemur/tree/master/prpy_lemur/
[prlab]: https://personalrobotics.ri.cmu.edu/
[ri]: http://www.ri.cmu.edu/
