Roadmaps {#or-lemur-roadmaps}
========

The `ompl_lemur` package includes the ompl_lemur::Roadmap class,
along with several implemented instantiations (e.g. grids, RGGs,
Halton graphs, etc).  See the [Roadmaps](@ref ompl-lemur-roadmaps)
page for details.

The `or_lemur` package includes some tools for working with roadmaps
over the C-space of robots in OpenRAVE.

Generating Roadmaps
-------------------

First, you need to determine the size of your space.
`or_lemur` includes the `get-robot-space-bounds.py` script:

    $ rosrun or_lemur get-robot-space-bounds.py
       --urdf=package://herb_description/robots/herb.urdf
       --srdf=package://herb_description/robots/herb.srdf --manip=right

The output is:

    Robot space bounds:
      dof [0] from 0.54159265359 to 5.74159265359 (joint /right/j1, range 5.2)
      dof [1] from -1.96 to 1.96 (joint /right/j2, range 3.92)
      dof [2] from -2.73 to 2.73 (joint /right/j3, range 5.46)
      dof [3] from -0.86 to 3.13 (joint /right/j4, range 3.99)
      dof [4] from -4.79 to 1.3 (joint /right/j5, range 6.09)
      dof [5] from -1.56 to 1.56 (joint /right/j6, range 3.12)
      dof [6] from -2.99 to 2.99 (joint /right/j7, range 5.98)
    Bounds command line (e.g. for ompl_lemur generate-roadmap):
      --dim=7 --bounds=0:0.54159265359,5.74159265359 --bounds=1:-1.96,1.96 --bounds=2:-2.73,2.73 --bounds=3:-0.86,3.13 --bounds=4:-4.79,1.3 --bounds=5:-1.56,1.56 --bounds=6:-2.99,2.99

The generated command line can then be passed to the
`generate-roadmap` command:

    $ rosrun ompl_lemur generate-roadmap
       --dim=7 --bounds=0:0.54159265359,5.74159265359 --bounds=1:-1.96,1.96 --bounds=2:-2.73,2.73 --bounds=3:-0.86,3.13 --bounds=4:-4.79,1.3 --bounds=5:-1.56,1.56 --bounds=6:-2.99,2.99
       --roadmap-type=HaltonOffDens
       --roadmap-param=num_per_batch=10000
       --roadmap-param=gamma_factor=1.0
       --roadmap-param=scaling=log_n
       --roadmap-param=seed=0
       --num-batches=1
       --out-file=my-roadmap.graphml --out-format=graphml

The resulting graph file can the be loaded into any program that
supports the [GraphML][graphml] format (e.g. [NetworkX][networkx] in
Python).

Saving Cached Roadmaps
----------------------

`or_lemur` includes a implementation of cached roadmaps via the or_lemur::RoadmapCached class.  This class stores cache files in `$OPENRAVE_HOME/or_lemur/roadmap-HASH.bin` files, where `HASH` is a function of both the space dimensions/bounds and the roadmap type/parameters.

Both the or_lemur::LEMUR and or_lemur::FamilyPlanner planners will use cached roadmaps if they are requested.  For example, to use the cached version of the `HaltonOffDens` roadmap type, pass the `CachedHaltonOffDens` type instead.

One way to generate a cache file is via the following helper script:

    $ rosrun or_lemur save-roadmap-cache.py
       --urdf=package://herb_description/robots/herb.urdf
       --srdf=package://herb_description/robots/herb.srdf --manip=right
       --roadmap-type=HaltonOffDens
       --roadmap-param=num_per_batch=10000
       --roadmap-param=gamma_factor=0.9
       --roadmap-param=scaling=log_n
       --roadmap-param=seed=0
       --num-batches=4

This will produce output which looks something like this:

    [RoadmapCached.h:107 initialize] Could not find cached roadmap file: or_lemur/roadmap-934dda6ac853e0c39f6cf12ad9c941c5.bin
    [LEMUR.cpp:1489] Densifying roadmap to batch [0] ...
    [LEMUR.cpp:1489] Densifying roadmap to batch [1] ...
    [LEMUR.cpp:1489] Densifying roadmap to batch [2] ...
    [LEMUR.cpp:1489] Densifying roadmap to batch [3] ...
    [planner_lemur.cpp:248 PlanPath] Saving cached roadmap ...
    [RoadmapCached.h:261 save_file] Saving file ...

In this example, the generated file is approximately 170MiB for four batches.

[graphml]: http://graphml.graphdrawing.org/
[networkx]: https://networkx.github.io/
