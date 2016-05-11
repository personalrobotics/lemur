/*! \file lazysp_incsp_incbi.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Adaptor to use pr_bgl::incbi as the inner sp algorithm for
 *        pr_bgl::lazysp.
 */

namespace pr_bgl
{

template<class Graph, class EdgeIndexMap>
class lazysp_incsp_incbi_edge_index_adaptor
{
public:
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::readable_property_map_tag category;
   typedef Edge key_type;
   typedef size_t value_type;
   typedef size_t reference;
   Graph & g;
   EdgeIndexMap edge_index_map;
   lazysp_incsp_incbi_edge_index_adaptor(Graph & g, EdgeIndexMap edge_index_map):
      g(g), edge_index_map(edge_index_map)
   {
   }
};

template<class Graph, class EdgeIndexMap>
inline const size_t get(
   const lazysp_incsp_incbi_edge_index_adaptor<Graph,EdgeIndexMap> & map,
   const typename boost::graph_traits<Graph>::edge_descriptor & e)
{
   return (get(map.edge_index_map,e) << 1)
      + ((source(e,map.g)<target(e,map.g)) ? 0 : 1);
}


/*! \brief Adaptor to use pr_bgl::incbi as the inner sp algorithm for
 *         pr_bgl::lazysp.
 * 
 * solve returns weight_type::max if a non-infinite path is found
 * 
 * solve is always called with the same g,v_start,v_goal
 * 
 * the edge indices that the inner incbi thinks its working with are
 * adapted:
\verbatim
the heap index that's used is the edge index << 1
with the lsb=0: lower vertex index is start-side
      or lsb=1: lower vertex index is goal-side
\endverbatim
 *
 * due to wincbi stuff, wmap is not necessarily symmetric!
 */
template <class Graph, class ActualWMap,
   class StartPredecessorMap, class StartDistanceMap, class StartDistanceLookaheadMap,
   class GoalPredecessorMap, class GoalDistanceMap, class GoalDistanceLookaheadMap,
   class EdgeIndexMap, class EdgeVectorMap,
   typename CompareFunction, typename CombineFunction,
   class IncBiVisitor>
class lazysp_incsp_incbi
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::property_traits<ActualWMap>::value_type weight_type;
   typedef typename boost::property_map<Graph, boost::vertex_index_t>::type VIndexMap;

   Graph & g;
   Vertex v_start;
   Vertex v_goal;
   ActualWMap w_map;
   StartPredecessorMap start_predecessor;
   StartDistanceMap start_distance;
   GoalPredecessorMap goal_predecessor;
   GoalDistanceMap goal_distance;
   EdgeVectorMap edge_vector_map;
   weight_type inf;
   
   // incbi instance
   pr_bgl::incbi<Graph,
      StartPredecessorMap,StartDistanceMap,StartDistanceLookaheadMap,
      GoalPredecessorMap,GoalDistanceMap,GoalDistanceLookaheadMap,
      ActualWMap,
      VIndexMap,
      lazysp_incsp_incbi_edge_index_adaptor<Graph,EdgeIndexMap>,
      //std::less<weight_type>, boost::closed_plus<weight_type>,
      CompareFunction, CombineFunction,
      weight_type, weight_type,
      IncBiVisitor, incbi_balancer_distance<Vertex,weight_type>
   > incbi;
   
   lazysp_incsp_incbi(
      Graph & g, Vertex v_start, Vertex v_goal, ActualWMap w_map,
      StartPredecessorMap start_predecessor,
      StartDistanceMap start_distance, StartDistanceLookaheadMap start_distance_lookahead,
      GoalPredecessorMap goal_predecessor,
      GoalDistanceMap goal_distance, GoalDistanceLookaheadMap goal_distance_lookahead,
      EdgeIndexMap edge_index_map,
      EdgeVectorMap edge_vector_map,
      weight_type goal_margin,
      CompareFunction compare, CombineFunction combine,
      weight_type inf, weight_type zero,
      IncBiVisitor vis):
      g(g), v_start(v_start), v_goal(v_goal),
      w_map(w_map),
      start_predecessor(start_predecessor),
      start_distance(start_distance),
      goal_predecessor(goal_predecessor),
      goal_distance(goal_distance),
      edge_vector_map(edge_vector_map),
      inf(inf),
      incbi(g, v_start, v_goal,
         start_predecessor, start_distance, start_distance_lookahead,
         goal_predecessor, goal_distance, goal_distance_lookahead,
         w_map,
         get(boost::vertex_index, g), // vertex_index_map
         lazysp_incsp_incbi_edge_index_adaptor<Graph,EdgeIndexMap>(g, edge_index_map),
         //std::less<weight_type>(), // compare
         //boost::closed_plus<weight_type>(std::numeric_limits<weight_type>::max()), // combine
         //std::numeric_limits<weight_type>::max(),
         //weight_type(),
         compare, combine, inf, zero,
         goal_margin,
         vis, incbi_balancer_distance<Vertex,weight_type>())
   {
   }
   
   template <typename LazySPWMap>
   weight_type solve(const Graph & g, Vertex v_start, Vertex v_goal,
      LazySPWMap wmap, std::vector<Edge> & path)
   {
      std::pair<size_t,bool> spresult = incbi.compute_shortest_path();
      
      // no solution?
      if (!spresult.second)
         return inf;
      
      // which way does the edge go?
      size_t eidx_actual = spresult.first >> 1;
      Edge e = get(edge_vector_map, eidx_actual);
      
      Vertex v_startside = source(e, g);
      Vertex v_goalside = target(e, g);
      unsigned char goalside_is_lower = (v_goalside < v_startside) ? 1 : 0;
      unsigned char goalside_shouldbe_lower = spresult.first & 1; // 0 or 1
      
      if (goalside_is_lower ^ goalside_shouldbe_lower)
      {
         Vertex v_temp = v_startside;
         v_startside = v_goalside;
         v_goalside = v_temp;
         e = edge(target(e,g),source(e,g),g).first;
      }
      
      // get path
      path.clear();
      // first, walk from the startside vertex to the start
      for (Vertex v_walk=v_startside; v_walk!=v_start;)
      {
         Vertex v_pred = get(start_predecessor,v_walk);
         std::pair<Edge,bool> ret = edge(v_pred, v_walk, g);
         BOOST_ASSERT(ret.second);
         path.push_back(ret.first);
         v_walk = v_pred;
      }
      std::reverse(path.begin(),path.end());
      // then add middle edge
      {
         std::pair<Edge,bool> ret = edge(v_startside, v_goalside, g);
         BOOST_ASSERT(ret.second);
         path.push_back(ret.first);
      }
      // then, walk from the goalside vertex to the goal
      for (Vertex v_walk=v_goalside; v_walk!=v_goal;)
      {
         Vertex v_succ = get(goal_predecessor,v_walk);
         std::pair<Edge,bool> ret = edge(v_walk, v_succ, g);
         BOOST_ASSERT(ret.second);
         path.push_back(ret.first);
         v_walk = v_succ;
      }
      
      return get(start_distance,v_startside) + get(w_map,e) + get(goal_distance,v_goalside);
   }
   
   void update_notify(Edge euv)
   {
      Vertex u = source(euv,g);
      Vertex v = target(euv,g);
      // update forward edge
      weight_type wuv = get(w_map,euv);
      incbi.start_update_predecessor(u, v, wuv);
      incbi.start_update_vertex(v);
      incbi.goal_update_successor(u, v, wuv);
      incbi.goal_update_vertex(u);
      incbi.update_edge(euv);
      // update reverse edge
      Edge evu = edge(v,u,g).first;
      weight_type wvu = get(w_map,evu);
      incbi.start_update_predecessor(v, u, wvu);
      incbi.start_update_vertex(u);
      incbi.goal_update_successor(v, u, wvu);
      incbi.goal_update_vertex(v);
      incbi.update_edge(evu);
   }
};

template <class Graph, class ActualWMap,
   class StartPredecessorMap, class StartDistanceMap, class StartDistanceLookaheadMap,
   class GoalPredecessorMap, class GoalDistanceMap, class GoalDistanceLookaheadMap,
   class EdgeIndexMap, class EdgeVectorMap,
   typename CompareFunction, typename CombineFunction,
   class IncBiVisitor>
lazysp_incsp_incbi<Graph,ActualWMap,StartPredecessorMap,StartDistanceMap,StartDistanceLookaheadMap,GoalPredecessorMap,GoalDistanceMap,GoalDistanceLookaheadMap,EdgeIndexMap,EdgeVectorMap,CompareFunction,CombineFunction,IncBiVisitor>
make_lazysp_incsp_incbi(
   Graph & g,
   typename boost::graph_traits<Graph>::vertex_descriptor v_start,
   typename boost::graph_traits<Graph>::vertex_descriptor v_goal,
   ActualWMap w_map,
   StartPredecessorMap start_predecessor,
   StartDistanceMap start_distance, StartDistanceLookaheadMap start_distance_lookahead,
   GoalPredecessorMap goal_predecessor,
   GoalDistanceMap goal_distance, GoalDistanceLookaheadMap goal_distance_lookahead,
   EdgeIndexMap edge_index_map, EdgeVectorMap edge_vector_map,
   typename boost::property_traits<ActualWMap>::value_type goal_margin,
   CompareFunction compare, CombineFunction combine,
   typename boost::property_traits<ActualWMap>::value_type inf,
   typename boost::property_traits<ActualWMap>::value_type zero,
   IncBiVisitor vis)
{
   return lazysp_incsp_incbi<Graph,ActualWMap,StartPredecessorMap,StartDistanceMap,StartDistanceLookaheadMap,GoalPredecessorMap,GoalDistanceMap,GoalDistanceLookaheadMap,EdgeIndexMap,EdgeVectorMap,CompareFunction,CombineFunction,IncBiVisitor>(
      g, v_start, v_goal, w_map, start_predecessor, start_distance, start_distance_lookahead, goal_predecessor, goal_distance, goal_distance_lookahead, edge_index_map, edge_vector_map, goal_margin, compare, combine, inf, zero, vis);
}

} // namespace pr_bgl
