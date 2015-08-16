/* File: overlay_manager.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace pr_bgl
{

// ok, so we want to maintain a second graph
// with persistent vertex descriptors (e.g. using linked lists)
// doesnt handle properties (you do this all yourself)
// eventually, should i have it auto-copy them somehow?
template <class GCore, class VertexIndexMap, class EdgeIndexMap, class EdgeVectorMap,
   class GOver, class OverVertexMap, class OverEdgeMap>
class OverlayManager
{
public:

   GCore & gcore;
   VertexIndexMap vertex_index_map;
   EdgeIndexMap edge_index_map;
   EdgeVectorMap edge_vector_map;
   GOver & gover;
   OverVertexMap over_vertex_map;
   OverEdgeMap over_edge_map;
   OverlayManager(
         GCore & gcore,
         VertexIndexMap vertex_index_map,
         EdgeIndexMap edge_index_map,
         EdgeVectorMap edge_vector_map,
         GOver & gover,
         OverVertexMap over_vertex_map,
         OverEdgeMap over_edge_map):
      gcore(gcore),
      vertex_index_map(vertex_index_map),
      edge_index_map(edge_index_map),
      edge_vector_map(edge_vector_map),
      gover(gover),
      over_vertex_map(over_vertex_map),
      over_edge_map(over_edge_map)
   {
   }
   
   // keep track of order of vertices and edges that have been applied
   std::vector<typename boost::graph_traits<GOver>::vertex_descriptor> applied_vertices;
   std::vector<typename boost::graph_traits<GOver>::edge_descriptor> applied_edges;
   
   void apply()
   {
      // copy non-anchor VERTICES first
      typename boost::graph_traits<GOver>::vertex_iterator vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(gover); vi!=vi_end; ++vi)
      {
         // skip anchor vertices (they're already added, no need to copy anything either)
         if (get(over_vertex_map, *vi) != boost::graph_traits<GCore>::null_vertex())
            continue;
         // ok, need to add this vertex to core graph!
         typename boost::graph_traits<GCore>::vertex_descriptor vcore = add_vertex(gcore);
         // set vertex index? (only for non-adjacecy list)
         put(over_vertex_map, *vi, vcore);
         applied_vertices.push_back(*vi);
         // copy data?
      }
      
      // compute number of edges currently in core graph
      // so we can set the edge index map correctly (ugh!)
      unsigned int num_core_edges = 0;
      {
         typename boost::graph_traits<GCore>::edge_iterator ei, ei_end;
         for (boost::tie(ei,ei_end)=edges(gcore); ei!=ei_end; ++ei)
            num_core_edges++;
      }
      
      // next, copy all edges
      typename boost::graph_traits<GOver>::edge_iterator ei, ei_end;
      for (boost::tie(ei,ei_end)=edges(gover); ei!=ei_end; ++ei)
      {
         // ok, need to add this edge to core graph!
         typename boost::graph_traits<GCore>::edge_descriptor ecore
            = add_edge(get(over_vertex_map, source(*ei,gover)),
                       get(over_vertex_map, target(*ei,gover)), gcore).first;
         put(over_edge_map, *ei, ecore);
         applied_edges.push_back(*ei);
         // extra edge index bookkeeping
         put(edge_index_map, ecore, num_core_edges);
         put(edge_vector_map, num_core_edges, ecore);
         num_core_edges++;
         // copy data?
      }
   }
   
   void unapply()
   {
      // remove edges in reverse order
      while (applied_edges.size())
      {
         typename boost::graph_traits<GOver>::edge_descriptor eover = applied_edges.back();
         applied_edges.pop_back();
         // TODO: edge index bookkeeping???
         // map from edge to index is probably backed by bundled property, so itll get removed automatically
         // map from index to edge is a separate vector ... how to remove this?
         // TODO: copy data back?
         typename boost::graph_traits<GCore>::edge_descriptor ecore = get(over_edge_map, eover);
         remove_edge(ecore, gcore);
      }
      // remove vertices in reverse order
      while (applied_vertices.size())
      {
         typename boost::graph_traits<GOver>::vertex_descriptor vover = applied_vertices.back();
         applied_vertices.pop_back();
         // TODO: vertex index bookkeeping???
         // TODO: copy data back?
         typename boost::graph_traits<GCore>::vertex_descriptor vcore = get(over_vertex_map, vover);
         remove_vertex(vcore, gcore);
         // reset vertex pointer so over vertex is known to be a non-anchor vertex
         put(over_vertex_map, vover, boost::graph_traits<GCore>::null_vertex());
      }
   }
};

} // namespace pr_bgl
