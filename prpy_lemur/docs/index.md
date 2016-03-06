prpy_lemur: LEMUR for PrPy {#index}
==========================

[`prpy`][github-prpy] bindings for Chris Dellin's LEMUR planner.  For
now, only a few features are supported.  For example, caching between
calls (within a single planner instance, or through cached files) is
not yet supported.

Usage
-----

    # Create a prpy planner instance
    import prpy_lemur.planning_multiset
    planner = prpy_lemur.planning_multiset.MultisetPlanner()

    # Plan
    traj = planner.PlanToConfiguration(robot, q_goal)

Planning Methods Supported
--------------------------

We don't currently support many planning methods.  Here is a list:

- `PlanToConfiguration(robot, goal_config)`: SUPPORTED
- `PlanToConfigurations(robot, goal_configs)`: UNSUPPORTED
- `PlanToEndEffectorPose(robot, goal_pose)`: UNSUPPORTED
- `PlanToEndEffectorOffset(robot, direction, min_distance, max_distance)` UNSUPPORTED
- `PlanToTSR(robot, tsrchains)`: UNSUPPORTED
- `PlanToBasePose(robot, goal_pose)`: UNSUPPORTED

License
-------

* Source code for this package is on [GitHub][github-prpy-lemur].

LEMUR is written by Chris Dellin at the [Personal Robotics Lab][prlab]
at the [Robotics Intitute][ri], [Carnegie Mellon University][cmu].  It
was released on April 9, 2015 under a BSD license.

[cmu]: http://www.cmu.edu/
[github-prpy]: https://github.com/personalrobotics/prpy/
[github-prpy-lemur]: https://github.com/personalrobotics/lemur/tree/master/prpy_lemur/
[prlab]: https://personalrobotics.ri.cmu.edu/
[ri]: http://www.ri.cmu.edu/
