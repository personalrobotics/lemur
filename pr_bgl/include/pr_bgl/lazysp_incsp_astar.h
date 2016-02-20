/*! \file lazysp_incsp_astar.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Adaptor to use boost::astar_search as the inner sp
 *        algorithm for pr_bgl::lazysp.
 */

namespace pr_bgl
{

/*! \brief Adaptor to use boost::astar_search as the inner sp
 *         algorithm for pr_bgl::lazysp.
 * 
 * solve returns weight_type::max if a non-infinite path is found
 * 
 * solve is always called with the same g,v_start,v_goal
 */
template <class Graph, class WMap, class HeuristicMap, class PredecessorMap, class DistanceMap, class CostMap, class ColorMap>
class lazysp_incsp_astar
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
      inline void discover_vertex(Vertex u, const Graph & g) {}
      inline void examine_vertex(Vertex u, const Graph & g)
      {
         if (u == v_throw)
            throw throw_visitor_exception();
      }
      inline void examine_edge(Edge e, const Graph & g) {}
      inline void edge_relaxed(Edge e, const Graph & g) {}
      inline void edge_not_relaxed(Edge e, const Graph & g) {}
      inline void black_target(Edge e, const Graph & g) {}
      inline void finish_vertex(Vertex u, const Graph & g) {}
   };
   
   class map_heuristic
   {
   public:
      HeuristicMap heuristic_map;
      map_heuristic(HeuristicMap heuristic_map): heuristic_map(heuristic_map) {}
      inline weight_type operator()(Vertex u)
      {
         return get(heuristic_map, u);
      } 
   };
   
   HeuristicMap heuristic_map;
   PredecessorMap predecessor_map;
   DistanceMap distance_map;
   CostMap cost_map;
   ColorMap color_map;
   
   lazysp_incsp_astar(HeuristicMap heuristic_map, PredecessorMap predecessor_map, DistanceMap distance_map, CostMap cost_map, ColorMap color_map):
      heuristic_map(heuristic_map), predecessor_map(predecessor_map), distance_map(distance_map), cost_map(cost_map), color_map(color_map)
   {}
   
   weight_type solve(const Graph & g, Vertex v_start, Vertex v_goal,
      WMap wmap, std::vector<Edge> & path)
   {
      try
      {
         boost::astar_search(
            g,
            v_start,
            map_heuristic(heuristic_map), // AStarHeuristic h
            throw_visitor(v_goal), // AStarVisitor vis
            predecessor_map, // PredecessorMap predecessor
            cost_map, // CostMap cost
            distance_map, // DistanceMap distance
            wmap, // WeightMap weight
            get(boost::vertex_index, g), // VertexIndexMap index_map
            color_map, // ColorMap color
            std::less<weight_type>(), // compare
            boost::closed_plus<weight_type>(std::numeric_limits<weight_type>::max()), // combine
            std::numeric_limits<weight_type>::max(), // cost inf
            weight_type() // cost zero
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

template <class Graph, class WMap, class HeuristicMap, class PredecessorMap, class DistanceMap, class CostMap, class ColorMap>
lazysp_incsp_astar<Graph,WMap,HeuristicMap,PredecessorMap,DistanceMap,CostMap,ColorMap>
make_lazysp_incsp_astar(HeuristicMap heuristic_map, PredecessorMap predecessor_map, DistanceMap distance_map, CostMap cost_map, ColorMap color_map)
{
   return lazysp_incsp_astar<Graph,WMap,HeuristicMap,PredecessorMap,DistanceMap,CostMap,ColorMap>(heuristic_map, predecessor_map, distance_map, cost_map, color_map);
}

} // namespace pr_bgl
