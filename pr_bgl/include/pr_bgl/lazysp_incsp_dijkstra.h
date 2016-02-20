/*! \file lazysp_incsp_dijkstra.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Adaptor to use boost::dijkstra_shortest_paths as the inner sp
 *        algorithm for pr_bgl::lazysp.
 */

namespace pr_bgl
{

/*! \brief Adaptor to use boost::dijkstra_shortest_paths as the inner
 *         sp algorithm for pr_bgl::lazysp.
 * 
 * solve returns weight_type::max if a non-infinite path is found
 * 
 * solve is always called with the same g,v_start,v_goal
 */
template <class Graph, class WMap, class PredecessorMap, class DistanceMap>
class lazysp_incsp_dijkstra
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::property_traits<WMap>::value_type weight_type;
   
   struct throw_visitor_exception {};
   class throw_visitor
   {
   public:
      Vertex v_throw;
      throw_visitor(Vertex v_throw): v_throw(v_throw) {}
      inline void initialize_vertex(Vertex u, const Graph & g) {}
      inline void examine_vertex(Vertex u, const Graph & g)
      {
         if (u == v_throw)
            throw throw_visitor_exception();
      }
      inline void examine_edge(Edge e, const Graph & g) {}
      inline void discover_vertex(Vertex u, const Graph & g) {}
      inline void edge_relaxed(Edge e, const Graph & g) {}
      inline void edge_not_relaxed(Edge e, const Graph & g) {}
      inline void finish_vertex(Vertex u, const Graph & g) {}
   };
   
   PredecessorMap predecessor_map;
   DistanceMap distance_map;
   
   lazysp_incsp_dijkstra(PredecessorMap predecessor_map, DistanceMap distance_map):
      predecessor_map(predecessor_map), distance_map(distance_map)
   {
   }
   
   weight_type solve(const Graph & g, Vertex v_start, Vertex v_goal,
      WMap wmap, std::vector<Edge> & path)
   {
      try
      {
         boost::dijkstra_shortest_paths(
            g,
            v_start,
            predecessor_map,
            distance_map,
            wmap,
            get(boost::vertex_index, g), // implicit vertex index map
            std::less<weight_type>(), // compare
            boost::closed_plus<weight_type>(std::numeric_limits<weight_type>::max()), // combine
            std::numeric_limits<weight_type>::max(), // cost inf
            weight_type(), // cost zero
            throw_visitor(v_goal)
            //boost::make_dijkstra_visitor(boost::null_visitor())
         );
      }
      catch (const throw_visitor_exception & ex)
      {
      }
         
      if (get(distance_map,v_goal) == std::numeric_limits<weight_type>::max())
         return std::numeric_limits<weight_type>::max();
      
      // get path
      path.clear();
      for (Vertex v_walk=v_goal; v_walk!=v_start;)
      {
         Vertex v_pred = get(predecessor_map, v_walk);
         std::pair<Edge,bool> ret = edge(v_pred, v_walk, g);
         BOOST_ASSERT(ret.second);
         path.push_back(ret.first);
         v_walk = v_pred;
      }
      std::reverse(path.begin(),path.end());
      
      return get(distance_map, v_goal);
   }
   
   void update_notify(Edge e)
   {
   }
};

template <class Graph, class WMap, class PredecessorMap, class DistanceMap>
lazysp_incsp_dijkstra<Graph,WMap,PredecessorMap,DistanceMap>
make_lazysp_incsp_dijkstra(PredecessorMap predecessor_map, DistanceMap distance_map)
{
   return lazysp_incsp_dijkstra<Graph,WMap,PredecessorMap,DistanceMap>(predecessor_map, distance_map);
}

} // namespace pr_bgl
