Family Planning {#or-lemur-family-planning}
===============

Family Module
-------------

The or_lemur::FamilyModule class is an OpenRAVE module which manages
a family of sets over the C-space of a robot.

Family Planner
--------------

The or_lemur::FamilyPlanner is an OpenRAVE planner which uses a family
module instance to generate the relations and indicator functions for
each set.  In addition to the parameters accepted by the
[LEMUR Planner](@ref or-lemur-lemur-planner),
the or_lemur::FamilyPlanner requires the instance ID of the
family module that it should use.

Scripts
-------

The `or_ompl` package includes a script `family-self-setcache.py` which
wraps a or_lemur::FamilyPlanner instance and computes a self-collision
checked set cache for a particular roadmap.  Here's an example of
computing such a set cache for the HERB robot:

    $ rosrun or_lemur family-self-setcache.py
       --urdf=package://herb_description/robots/herb.urdf
       --srdf=package://herb_description/robots/herb.srdf --manip=right
       --collision-checker=fcl
       --roadmap-type=HaltonOffDens
       --roadmap-param=num_per_batch=10000
       --roadmap-param=radius_first_batch=2.0
       --roadmap-param=seed=0
       --num-batches=1
       --setcache=setcache-herbright-self-halton00.txt

