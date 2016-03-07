ompl_lemur: LEMUR for OMPL {#index}
==========================

The `ompl_lemur` package provides the core implementation of the LEMUR
motion planner: Lazily Evaluated Marginal Utility Roadmaps.

`ompl_lemur` is part of the LEMUR repository (hosted on
[github][github-lemur]) which includes these packages:

* **ompl_lemur** - implementation of LEMUR for [OMPL][ompl] (this
   package)
* [or_lemur](../or_lemur/index.html) - LEMUR bindings for
   [OpenRAVE][openrave]
* [pr_bgl](../pr_bgl/index.html) - graph search algorithms for the
   [Boost Graph Library][bgl]
* [prpy_lemur](../prpy_lemur/index.html) - LEMUR bindings for
   [PrPy][github-prpy]

Planner
-------

LEMUR is implemented in C++ as a planner for the
[Open Motion Planning Library (OMPL)][ompl] via the ompl_lemur::LEMUR
class.  A detailed description, as well as usage examples, are
provided in the [LEMUR Planner](@ref ompl-lemur-lemur-planner) page.

Roadmaps
--------

ompl_lemur::LEMUR uses the [Boost Graph Library (BGL)][bgl] for roadmap
representation and core search algorithms.  Additional algorithms for
BGL graphs are implemented in the `pr_bgl` package in the [LEMUR
repository][github-lemur].

`ompl_lemur` provides a facility to create several common types of BGL
roadmaps over an OMPL state space using the ompl_lemur::Roadmap class.
Types of roadmaps currently implemented include Random Geometric Graphs
(RGGs) and Halton graphs.
See the [Roadmaps](@ref ompl-lemur-roadmaps) page for more information about
the various roadmap types.

Planning over C-Space Families
------------------------------

See the [Family Planning](@ref ompl-lemur-family-planning) page for more
information about the family planning code.

License
-------

LEMUR is written by Chris Dellin at the [Personal Robotics Lab][prlab]
at the [Robotics Intitute][ri], [Carnegie Mellon University][cmu].  It
was released on April 9, 2015 under a BSD license.

Source code for this package is on [GitHub][github-ompl-lemur].

[bgl]: http://www.boost.org/doc/libs/release/libs/graph/
[cmu]: http://www.cmu.edu/
[github-lemur]: https://github.com/personalrobotics/lemur
[github-ompl-lemur]: https://github.com/personalrobotics/lemur/tree/master/ompl_lemur/
[github-prpy]: https://github.com/personalrobotics/prpy/
[ompl]: http://ompl.kavrakilab.org/
[openrave]: http://openrave.org/
[prlab]: https://personalrobotics.ri.cmu.edu/
[ri]: http://www.ri.cmu.edu/
