Multi-Set Planning
==================

This repository holds Chris Dellin's experimental planner, known as the multiset planner, e8, or family planner.  This code was written at the [Personal Robotics Lab][pr] at the [Robotics Institute][ri] at [Carnegie Mellon University][cmu], and was released on Apr 9, 2015 under a BSD license.

Packages
--------

The code is split into several ROS packages:

### [`pr_bgl`](pr_bgl/)

Implementations of graph algorithms for the Boost Graph Library.

Dependencies: `boost`

### [`ompl_multiset`](ompl_multiset/)

Core planner implementation for the [Open Motion Planning Library (OMPL)][ompl].  Uses [Boost Graph][bgl] for the implementation.

Dependencies: `ompl`

### [`or_multiset`](or_multiset/)

Bindings for the [OpenRAVE][openrave] planning environment.  A part of this package currently duplicates [`or_ompl`][orompl].

Dependencies: `ompl`, `openrave`, `ompl_multiset`

### [`prpy_multiset`](prpy_multiset/)

Bindings for the [PrPy][prpy] python library. 

Dependencies: `prpy`, `or_multiset`

### [`test_multiset`](test_multiset/)

Various test scripts for the above packages.

Dependencies: `ompl`, `openrave`, `ompl_multiset`, `or_multiset`

[bgl]: http://www.boost.org/doc/libs/release/libs/graph/
[cmu]: http://www.cmu.edu/
[ompl]: http://ompl.kavrakilab.org/
[openrave]: http://openrave.org/
[orompl]: https://github.com/personalrobotics/or_ompl
[pr]: https://personalrobotics.ri.cmu.edu/
[prpy]: https://github.com/personalrobotics/prpy/
[ri]: http://www.ri.cmu.edu/