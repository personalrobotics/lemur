LEMUR
=====

[![Documentation](https://readthedocs.org/projects/lemur-planning/badge/?version=latest)](http://lemur-planning.readthedocs.org/en/latest/?badge=latest)

This repository holds Chris Dellin's experimental motion planner LEMUR: **Lazily Evaluated Marginal Utility Roadmaps**.  This code was written at the [Personal Robotics Lab][pr] at the [Robotics Institute][ri] at [Carnegie Mellon University][cmu], and was released on Apr 9, 2015 under a BSD license.  It is currently under active development.  Previous versions of this planner were known as the multiset planner, E8, or family planner.

Packages
--------

The code is split into several ROS packages:

### [`pr_bgl`](pr_bgl/)

Generic implementations of graph algorithms for the [Boost Graph Library (BGL)][bgl].  Includes implemetations of lazy search (`LazySP`), incremental search (Lifelong Planning A*, incremental bidirectional Dijkstra's), partition functions over graphs, and various helpers for BGL and [Boost property maps][bpropmap].

Dependencies: `boost`

### [`ompl_lemur`](ompl_lemur/)

Core planner implementation for the [Open Motion Planning Library (OMPL)][ompl].  Uses BGL and `pr_bgl` for the implementation.  Includes a `Roadmap` class, with implemetations for lattices, random geometric graphs (RGGs), Halton graphs, and densified versions thereof.

Dependencies: `ompl`, `pr_bgl`

### [`or_lemur`](or_lemur/)

Bindings for the [OpenRAVE][openrave] planning environment.  Related to [`or_ompl`][orompl].  Includes an OpenRAVE module for managing and relating free C-space subsets induced by collision checking.

Dependencies: `ompl`, `openrave`, `ompl_lemur`

### [`prpy_lemur`](prpy_lemur/)

Planning bindings for the [PrPy][prpy] python library.

Dependencies: `prpy`, `or_lemur`

### [`test_lemur`](test_lemur/)

Various test scripts for the above packages.

Dependencies: `ompl`, `openrave`, `ompl_lemur`, `or_lemur`

[bgl]: http://www.boost.org/doc/libs/release/libs/graph/
[bpropmap]: http://www.boost.org/doc/libs/release/libs/property_map
[cmu]: http://www.cmu.edu/
[ompl]: http://ompl.kavrakilab.org/
[openrave]: http://openrave.org/
[orompl]: https://github.com/personalrobotics/or_ompl
[pr]: https://personalrobotics.ri.cmu.edu/
[prpy]: https://github.com/personalrobotics/prpy/
[ri]: http://www.ri.cmu.edu/
