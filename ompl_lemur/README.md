`ompl_lemur`
============

Core planner implementation for the [Open Motion Planning Library (OMPL)][ompl].  Uses [Boost Graph][bgl] for the implementation.  Uses helpers for Boost Graph from the `pr_bgl` package.

Planners
--------

The primary planner is `LEMUR`, a derived class of `ompl::base::Planner`.  It takes the following arguments:

* `const ompl::base::SpaceInformationPtr & si`
* `ompl_lemur::EffortModel & effort_model`,
* `ompl_lemur::TagCache & tag_cache`

The roadmap to be searched is an instance of a roadmap type which subclasses `ompl_lemur::Roadmap`.  Some of the classes implemented are described in the [Roadmaps](#roadmaps) section below.

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

Roadmaps
--------

This package contains classes called "roadmap generators" which generate roadmaps from OMPL state spaces into Boost Graph objects.  Here is a quick tour of roadmap types implemented so far:

### `RoadmapGenAAGrid`

Axis aligned grids.  One level.  Arguments: `res=0.16`.

![roadmap-aagrid-res0.16](img/roadmap-aagrid-res0.16.png)

### `RoadmapGenHalton`

Halton sequences with fixed connection radii.  One level.  Arguments: `n=30 radius=0.3`.

![roadmap-halton-n30-radius0.3](img/roadmap-halton-n30-radius0.3.png)

### `RoadmapGenHaltonDens`

Densified, batched Halton sequences with connection radii reduced at each batch according to dispersion estimate.  Infinite levels.  Arguments: `n_perbatch=30 radius_firstbatch=0.3`.  Shown here up to three subgraphs.

![roadmap-haltondens-nperbatch30-radiusfirstbatch0.3](img/roadmap-haltondens-nperbatch30-radiusfirstbatch0.3.png)

### `RoadmapGenRGG`

Random geometric graphs.  One level.  Arguments: `n=30 radius=0.3 seed=1`.

![roadmap-rgg-n30-radius0.3-seed1](img/roadmap-rgg-n30-radius0.3-seed1.png)

[bgl]: http://www.boost.org/doc/libs/release/libs/graph/
[ompl]: http://ompl.kavrakilab.org/
