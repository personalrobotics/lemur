or_lemur: LEMUR for OpenRAVE {#index}
============================
@anchor or-lemur-index

The `or_lemur` package contains LEMUR bindings for [OpenRAVE][openrave].
The primary interface is through the or_lemur::LEMUR class, as
described on the [LEMUR Planner](@ref or-lemur-lemur-planner) page.

This planner also provides a facility to automatically discover C-space
families induced by geometric collision checking.  The interface for
this functionality is through the or_lemur::FamilyModule class.
Once a family has been defined, it can be planned over using the
or_lemur::FamilyPlanner planner.

Dependencies: `boost`

License
-------

LEMUR is written by Chris Dellin at the [Personal Robotics Lab][prlab]
at the [Robotics Intitute][ri], [Carnegie Mellon University][cmu].  It
was released on April 9, 2015 under a BSD license.

Source code for this package is on [GitHub][github-or-lemur].

[cmu]: http://www.cmu.edu/
[github-or-lemur]: https://github.com/personalrobotics/lemur/tree/master/or_lemur/
[openrave]: http://openrave.org/
[prlab]: https://personalrobotics.ri.cmu.edu/
[ri]: http://www.ri.cmu.edu/
