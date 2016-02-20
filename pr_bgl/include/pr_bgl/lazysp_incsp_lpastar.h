/*! \file lazysp_incsp_lpastar.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Adaptor to use pr_bgl::lpastar as the inner sp algorithm for
 *        pr_bgl::lazysp.
 */

namespace pr_bgl
{

/*! \brief Adaptor to use pr_bgl::lpastar as the inner sp algorithm for
 *         pr_bgl::lazysp.
 * 
 * solve returns weight_type::max if a non-infinite path is found
 * 
 * solve is always called with the same g,v_start,v_goal
 */
template <class Graph, class WMap, class HeuristicMap, class PredecessorMap, class DistanceMap, class DistanceLookaheadMap>
class lazysp_incsp_lpastar
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::property_traits<WMap>::value_type weight_type;
   typedef typename boost::property_map<Graph, boost::vertex_index_t>::type VIndexMap;
   
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
   
   Graph & g;
   Vertex v_start;
   Vertex v_goal;
   PredecessorMap predecessor_map;
   DistanceMap distance_map;
   
   // lpa* instance
   pr_bgl::lpastar<Graph,
      map_heuristic,
      boost::astar_visitor<boost::null_visitor>,
      PredecessorMap,
      DistanceMap,
      DistanceLookaheadMap,
      WMap,
      VIndexMap,
      std::less<weight_type>, boost::closed_plus<weight_type>,
      weight_type, weight_type
   > lpastar;
   
   lazysp_incsp_lpastar(
      Graph & g, Vertex v_start, Vertex v_goal, WMap w_map,
      HeuristicMap heuristic_map,
      PredecessorMap predecessor_map, DistanceMap distance_map,
      DistanceLookaheadMap distance_lookahead_map,
      weight_type goal_margin):
      g(g), v_start(v_start), v_goal(v_goal),
      predecessor_map(predecessor_map),
      distance_map(distance_map),
      lpastar(g, v_start, v_goal,
         map_heuristic(heuristic_map),
         boost::make_astar_visitor(boost::null_visitor()),
         predecessor_map,
         distance_map,
         distance_lookahead_map,
         w_map,
         get(boost::vertex_index, g), // index_map
         std::less<weight_type>(), // compare
         boost::closed_plus<weight_type>(std::numeric_limits<weight_type>::max()), // combine
         std::numeric_limits<weight_type>::max(),
         weight_type(),
         goal_margin)
   {
   }
   
   weight_type solve(const Graph & g, Vertex v_start, Vertex v_goal,
      WMap wmap, std::vector<Edge> & path)
   {
      // this stops at the goal vertex
      lpastar.compute_shortest_path();
      
      // no solution?
      if (get(distance_map,v_goal) == std::numeric_limits<weight_type>::max())
         return std::numeric_limits<weight_type>::max();
      
      // get path
      path.clear();
      for (Vertex v_walk=v_goal; v_walk!=v_start;)
      {
         Vertex v_pred = get(predecessor_map,v_walk);
         std::pair<Edge,bool> ret = edge(v_pred, v_walk, g);
         BOOST_ASSERT(ret.second);
         path.push_back(ret.first);
         v_walk = v_pred;
      }
      std::reverse(path.begin(),path.end());
      
      return get(distance_map,v_goal);
   }
   
   void update_notify(Edge e)
   {
      lpastar.update_vertex(source(e,g));
      lpastar.update_vertex(target(e,g));
   }
};

template <class Graph, class WMap, class HeuristicMap, class PredecessorMap, class DistanceMap, class DistanceLookaheadMap>
lazysp_incsp_lpastar<Graph,WMap,HeuristicMap,PredecessorMap,DistanceMap,DistanceLookaheadMap>
make_lazysp_incsp_lpastar(
   Graph & g,
   typename boost::graph_traits<Graph>::vertex_descriptor v_start,
   typename boost::graph_traits<Graph>::vertex_descriptor v_goal,
   WMap w_map, HeuristicMap heuristic_map,
   PredecessorMap predecessor_map, DistanceMap distance_map,
   DistanceLookaheadMap distance_lookahead_map,
   typename boost::property_traits<WMap>::value_type goal_margin)
{
   return lazysp_incsp_lpastar<Graph,WMap,HeuristicMap,PredecessorMap,DistanceMap,DistanceLookaheadMap>(
      g, v_start, v_goal, w_map, heuristic_map, predecessor_map, distance_map, distance_lookahead_map, goal_margin);
}

} // namespace pr_bgl
