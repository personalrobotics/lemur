/*! \file overlay_manager.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Class which maintains an overlay graph
 *        (pr_bgl::overlay_manager).
 */

namespace pr_bgl
{

/*! \brief Class which maintains an overlay graph.
 * 
 * The overlay_manager class maintains a graph overlay. In other words,
 * given a core graph and an overlay graph, the manager can temporarily
 * "apply" the overlay onto the core graph, copying the appropriate
 * vertices and edges from the overlay to the core graph. It then
 * remembers the applied vertices and edges, so that they can be
 * "unapplied" later (removed from the core graph in reverse order).
 * It is conjectured that this operation is safe on a core implemented
 * as an adjacency list without invalidating vertex descriptors on the
 * core graph (see mailing list [0]).
 * 
 * [0] http://lists.boost.org/boost-users/2015/08/84850.php
 * 
 * ok, so we want to maintain a second graph
 * with persistent vertex descriptors (e.g. using linked lists)
 * doesnt handle properties (you do this all yourself)
 * eventually, should i have it auto-copy them somehow?
 * applied_vertices does not include anchor vertices!
 */
template <class GCore, class GOver, class OverVertexMap, class OverEdgeMap>
class overlay_manager
{
public:

   GCore & gcore;
   GOver & gover;
   OverVertexMap over_vertex_map;
   OverEdgeMap over_edge_map;
   bool is_applied;
   
   overlay_manager(
         GCore & gcore,
         GOver & gover,
         OverVertexMap over_vertex_map,
         OverEdgeMap over_edge_map):
      gcore(gcore),
      gover(gover),
      over_vertex_map(over_vertex_map),
      over_edge_map(over_edge_map),
      is_applied(false)
   {
   }
   
   // keep track of order of vertices and edges that have been applied
   std::vector<typename boost::graph_traits<GOver>::vertex_descriptor> applied_vertices;
   std::vector<typename boost::graph_traits<GOver>::edge_descriptor> applied_edges;
   
   void apply()
   {
      assert(is_applied == false);
      // copy non-anchor VERTICES first
      typename boost::graph_traits<GOver>::vertex_iterator vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(gover); vi!=vi_end; ++vi)
      {
         // skip anchor vertices (they're already added, no need to copy anything either)
         if (get(over_vertex_map, *vi) != boost::graph_traits<GCore>::null_vertex())
            continue;
         // ok, need to add this vertex to core graph!
         typename boost::graph_traits<GCore>::vertex_descriptor vcore = add_vertex(gcore);
         put(over_vertex_map, *vi, vcore);
         applied_vertices.push_back(*vi);
         // copy data?
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
         // copy data?
      }
      is_applied = true;
   }
   
   void unapply()
   {
      assert(is_applied == true);
      // remove edges in reverse order
      while (applied_edges.size())
      {
         typename boost::graph_traits<GOver>::edge_descriptor eover = applied_edges.back();
         applied_edges.pop_back();
         // TODO: copy data back?
         typename boost::graph_traits<GCore>::edge_descriptor ecore = get(over_edge_map, eover);
         remove_edge(ecore, gcore);
      }
      // remove vertices in reverse order
      while (applied_vertices.size())
      {
         typename boost::graph_traits<GOver>::vertex_descriptor vover = applied_vertices.back();
         applied_vertices.pop_back();
         // TODO: copy data back?
         typename boost::graph_traits<GCore>::vertex_descriptor vcore = get(over_vertex_map, vover);
         remove_vertex(vcore, gcore);
         // reset vertex pointer so over vertex is known to be a non-anchor vertex
         put(over_vertex_map, vover, boost::graph_traits<GCore>::null_vertex());
      }
      is_applied = false;
   }
};

} // namespace pr_bgl
