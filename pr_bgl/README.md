`pr_bgl`
========

Author: Chris Dellin `<cdellin@gmail.com>`

Dependencies: `boost`

The `pr_bgl` package contains implemtations of various algorithms and utilities that build atop the [Boost Graph Library (BGL)][bgl].  As is customary with BGL, almost everything is implemented generically in headers.

Property Map Helpers
--------------------

BGL uses [Boost property maps][property-map] heavily to provide access to properties of graphs, vertices, and edges to algorithms in a generic way.

### `compose_property_map.hpp`

`compose_property_map` was written by Guillaume Pinot, owned by Eurodecision (2013), and released in Boost 1.54 under the Boost Software License v1.0.  It's copied here to users of older versions of Boost.

Test coverage: No.

### `flag_set_map.h`

The `FlagSetMap` class wraps an existing map by putting `true` to an ancillary map (with the same key) whenever the primary wrap is accessed (read or written).

Test coverage: No.

### `pair_index_map.h`

The `PairIndexMap` class is a readable boost property map which maps from a pair of indexable entities to a master index like C row major order 2d matrix indices.

Test coverage: No.

### `string_map.h`

The `StringMap` class implements a read-write map which wraps an existing property by allowing converting its values to and from the `std::string` instances.  It requires that the free functions `stringify_from_x()` and `stringify_to_x()` for any custom values.

Test coverage: No.

Graph Helpers
-------------

### `edge_indexed_graph.h`

The `EdgeIndexedGraph` class wraps an existing graph object, while additionally maintaining incrementing edge indices in supplied property maps.  The behavior is undefined if edges are not removed in the inverse order that they are added.

Test coverage: No.

### `graph_io.h`

The `[read|write]_graphio_[graph|properties]()` functions implement serializing and deserializing a graph and its properties to a custom textual graph format, which stores vertices, edges and properties one per line.

Test coverage: No.

### `overlay_manager.h`

The `OverlayManager` class maintains a graph overlay.  In other words, given a core graph and an overlay graph, the manager can temporarily "apply" the overlay onto the core graph, copying the appropriate vertices and edges from the overlay to the core graph.  It then remembers the applied vertices and edges, so that they can be "unapplied" later (removed from the core graph in reverse order).  It is conjectured that this operation is safe on a core implemented as an adjacency list without invalidating vertex descriptors on the core graph (see [mailing list][bgl-list-remove]).

Test coverage: No.

### `rev_edge_map.h`

The `RevEdgeMap` class is a readable boost property map which maps from reversed edges to original edges in a reversed graph.

Test coverage: No.

Pathfinding Algorithms
----------------------

### `incbi.h`

The `IncBi` class implements incremental bidirectional
Dijkstra's search over a graph for the single-source
single-sink shortest path problem.

Test coverage: Yes (not yet converted to `gtest`).

### `lazysp.h`

The `lazy_shortest_path` function implements the Lazy Shortest Path algorithm for the single-source single-sink shortest path problem.  It takes as an argument an `EvalStrategy` object which determins for the candidate path found at each iteration which edge(s) to select for evaluation.

Test coverage: No.

### `path_generator.h`

The `PathGenerator` class implements a generator of all simple paths solving the single-source single-sink problem in non-decreasing order of length.

Test coverage: No.

Other Graph Algorithms
----------------------

### `partition_all.h`

The `partition_all` function calculates the edge-weight partition function over all paths between every pair of vertices on a graph, via a recursive formulation which is linear in the number of edges in the graph.  

Test coverage: Yes.

### `partition_simple.h`

The `partition_simple` function calculates an approximation to the edge-weight partition function over all simple paths between two given vertices.  The implementation returns the value over all simple paths with total length below a given parameter.

Test coverage: Yes.

Other Data Structures
---------------------

### `heap_indexed.h`

The `HeapIndexed` class implements a binary min-heap with index lookups.  Elements are identified with an index value (e.g. [0,num_vertices)).  The heap also maintains a vector backing, wich each element at a particular location.

Test coverage: Yes.

[bgl]: http://boost.org/doc/libs/release/libs/graph/
[property-map]: http://www.boost.org/doc/libs/1_59_0/libs/property_map/
[bgl-list-remove]: http://lists.boost.org/boost-users/2015/08/84850.php