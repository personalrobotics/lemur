Roadmaps {#ompl-lemur-roadmaps}
========

The `ompl_lemur` package includes code for generating roadmaps over
OMPL state spaces represented as Boost graph objects.

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
