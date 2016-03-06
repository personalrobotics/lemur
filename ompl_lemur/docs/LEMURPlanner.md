LEMUR Planner
=============

Here is top-level documentation for how to use the implementation of LEMUR for OMPL.

The primary planner is `ompl_lemur::LEMUR`, a derived class of `ompl::base::Planner`.  It takes the following arguments:

* `const ompl::base::SpaceInformationPtr & si`
* `ompl_lemur::TagCache & tag_cache`

The roadmap to be searched is an instance of a roadmap type which
subclasses `ompl_lemur::Roadmap`.
Some of the classes implemented are described on the
[Roadmaps](Roadmaps.md) page.

For an overview, see the [main page](../README.md).

### Parameters

In addition to the construction arguments above, `LEMUR` takes the following parameters:

#### Roadmap

* `roadmap_type` (see `LEMUR::registerRoadmapType`)
* `roadmap.<roadmap_param>`: delegated to roadmap instance

#### Optimization coefficients

* `coeff_distance` (double)
* `coeff_checkcost` (double)
* `coeff_batch` (double)

These parameters specify the weights on the objective function optimized by the planner at each iteration.

#### Search type: `search_type` (string)

This parameter specifies which algorithm is used for the inner search performed by `LazySP`.  Currently supported values include:

* `dijkstras` (rooted at start)
* `astar` (rooted at start)
* `lpastar` (rooted at start)
* `incbi`

#### Evaluation selector type: `eval_type` (string)

This parameter specifies which edge selector is used to select edges for evaluation at each iteration.  Currently supported values include:

* `fwd`
* `rev`
* `alt`
* `bisect`
* `fwd_eval`
* `partition_all`
* `sp_indicator_probability`

#### Other parameters

* `do_timing` (bool): Set to `true` to have the planner profile and report time spent during search and during edge evaluation.
* `persist_roots` (bool): Set to `true` to keep vertices and edges from previous problem definitions
* `num_batches_init` (int): number of batches to generate before proceeding with search
* `max_batches` (int): planner terminates after search fails over this number of batches generated
