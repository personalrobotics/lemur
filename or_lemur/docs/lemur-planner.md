LEMUR Planner {#or-lemur-lemur-planner}
=============

The or_lemur::LEMUR planner has the name "LEMUR", and can be called
like any other OpenRAVE planner.  For example:

    params = openravepy.Planner.PlannerParameters()
    params.SetRobotActiveJoints(robot)
    params.SetGoalConfig(q_goal)
    params.SetExtraParameters(
       '<roadmap_type>Halton</roadmap_type>'
       + '<roadmap_param>num=1000</roadmap_param>'
       + '<roadmap_param>radius=2.0</roadmap_param>')
    
    planner = openravepy.RaveCreatePlanner(env, 'LEMUR')
    planner.InitPlan(robot, params)
    
    traj = openravepy.RaveCreateTrajectory(env, '')
    result = planner.PlanPath(traj)

Roadmap Parameters
------------------

or_lemur::LEMUR supports the roadmap types defined in the
[ompl_lemur package](@ref ompl-lemur-roadmaps).  One of the following
strings can be passed in the `<roadmap_type>` parameter:

* `AAGrid`: ompl_lemur::RoadmapAAGrid
* `FromFile`: ompl_lemur::RoadmapFromFile
* `Halton`: ompl_lemur::RoadmapHalton
* `HaltonDens`: ompl_lemur::RoadmapHaltonDens
* `HaltonOffDens`: ompl_lemur::RoadmapHaltonOffDens
* `RGG`: ompl_lemur::RoadmapRGG
* `RGGDens`: ompl_lemur::RoadmapRGGDens
* `RGGDensConst`: ompl_lemur::RoadmapRGGDensConst

In addition, the following cached variants are provided through the
or_lemur::RoadmapCached class:

* `CachedAAGrid`
* `CachedHalton`
* `CachedHaltonDens`
* `CachedHaltonOffDens`
* `CachedRGG`
* `CachedRGGDens`
* `CachedRGGDensConst`

Each of these roadmap types require a number of parameters, which are
provided by the `<roadmap_param>` parameter.  A list of the required
parameters is provided on `ompl_lemur`'s
[Roadmaps](@ref ompl-lemur-roadmaps) page.

Parameters for ompl_lemur::LEMUR
================================

Many parameters are passed directly to the ompl_lemur::LEMUR instance
that is created on each call to or_lemur::LEMUR::InitPlan().
These parameters are documented on `ompl_lemur`'s
[LEMUR Planner](@ref ompl-lemur-lemur-planner) page.

* `<coeff_distance>` (float)
* `<coeff_checkcost>` (float)
* `<coeff_batch>` (float)
* `<do_timing>` (bool, `"true"` or `"false"`)
* `<persist_roots>` (bool, `"true"` or `"false"`)
* `<num_batches_init>` - int
* `<max_batches>` - int
* `<search_type>` (string)
* `<eval_type>` (string)

Additional Parameters
=====================

* `<do_roadmap_save>` (bool, `"true"` or `"false"`)
* `<alglog>` (string)
* `<do_alglog_append>` (bool, `"true"` or `"false"`)
* `<graph>` (string)
* `<time_limit>` (float)
