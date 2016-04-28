Roadmaps {#prpy-lemur-roadmaps}
========

The LEMUR planner works on roadmaps constructed in the robot's
configuration space.  Each planning invocation requires the roadmap
to be specified.  The `prpy_lemur` package includes a `roadmaps` module,
which contains a simple named-tuple interface for constructing such
roadmaps.

    >>> import prpy_lemur.roadmaps
    >>> my_roadmap = prpy_lemur.roadmaps.Halton(num=1000, radius=2.0)
    >>> my_roadmap
    Halton(num=1000, radius=2.0)

This roadmap specification can then be passed to the LEMUR planner.
This can be done either on the planning method:

    traj = planner.PlanToConfiguration(robot, q_goal,
       roadmap=my_roadmap, max_batches=1)

Or at planner construction time, so that it is used as the default
roadmap for all its planning methods:

    planner = prpy_lemur.LEMURPlanner(roadmap=my_roadmap)

These roadmap types are supported by `prpy_lemur.roadmaps`:

* `AAGrid(res)`
* `FromFile(filename, root_radius)`
* `Halton(num, radius)`
* `HaltonDens(num_per_batch, radius_first_batch)`
* `HaltonOffDens(num_per_batch, radius_first_batch, seed)`
* `HaltonOffLLDens(num_per_batch, radius_first_batch, seed)`
* `RGG(num, radius, seed)`
* `RGGDens(num_per_batch, radius_first_batch, seed)`
* `RGGDensConst(num_per_batch, radius, seed)`

This is the meaning of the various parameters:

* `filename` - the filename to load the roadmap from
* `num` - the number of milestones in the roadmap
* `num_per_batch` - the number of milestones in each batch of the
  roadmap
* `res` - the resolution (e.g. in radians) between grid vertices
* `seed` - the random seed used for the roadmap
* `radius` - the connection radius used for roadmap edges
* `radius_first_batch` - the connection radius used for the first batch
  of roadmap edges; subsequent radii are shrunk according to the
  appropriate rule
* `root_radius` - the connection radius used for roadmap edges to
  root milestones (i.e. at start or goal configurations)

With the exception of `FromFile`, each roadmap `Type` also have an
associated cached version `CachedType`.  This roadmap type inherits the
parameters from its parent type, and also includes the following
additional parameter:

* `is_cache_required` - whether the planner should fail if the cache
  file is not found (False by default)
