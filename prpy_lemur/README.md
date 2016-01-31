`prpy_lemur`
===============

`prpy` bindings for Chris Dellin's LEMUR planner (aka multiset, e8, or family planner).  For now, only a few features are supported.  For example, caching between calls (within a single planner instance, or through cached files) is not yet supported.

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