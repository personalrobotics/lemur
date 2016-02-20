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
 */
template <class Graph, class WMap,
   class StartPredecessorMap, class StartDistanceMap, class StartDistanceLookaheadMap,
   class GoalPredecessorMap, class GoalDistanceMap, class GoalDistanceLookaheadMap,
   class EdgeIndexMap, class EdgeVectorMap,
   class IncBiVisitor>
class lazysp_incsp_incbi
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::property_traits<WMap>::value_type weight_type;
   typedef typename boost::property_map<Graph, boost::vertex_index_t>::type VIndexMap;

   Graph & g;
   Vertex v_start;
   Vertex v_goal;
   WMap w_map;
   StartPredecessorMap start_predecessor;
   StartDistanceMap start_distance;
   GoalPredecessorMap goal_predecessor;
   GoalDistanceMap goal_distance;
   EdgeVectorMap edge_vector_map;
   
   // incbi instance
   pr_bgl::incbi<Graph,
      StartPredecessorMap,StartDistanceMap,StartDistanceLookaheadMap,
      GoalPredecessorMap,GoalDistanceMap,GoalDistanceLookaheadMap,
      WMap,
      VIndexMap,
      lazysp_incsp_incbi_edge_index_adaptor<Graph,EdgeIndexMap>,
      std::less<weight_type>, boost::closed_plus<weight_type>,
      weight_type, weight_type,
      IncBiVisitor, incbi_balancer_distance<Vertex,weight_type>
   > incbi;
   
   lazysp_incsp_incbi(
      Graph & g, Vertex v_start, Vertex v_goal, WMap w_map,
      StartPredecessorMap start_predecessor,
      StartDistanceMap start_distance, StartDistanceLookaheadMap start_distance_lookahead,
      GoalPredecessorMap goal_predecessor,
      GoalDistanceMap goal_distance, GoalDistanceLookaheadMap goal_distance_lookahead,
      EdgeIndexMap edge_index_map,
      EdgeVectorMap edge_vector_map,
      weight_type goal_margin,
      IncBiVisitor vis):
      g(g), v_start(v_start), v_goal(v_goal),
      w_map(w_map),
      start_predecessor(start_predecessor),
      start_distance(start_distance),
      goal_predecessor(goal_predecessor),
      goal_distance(goal_distance),
      edge_vector_map(edge_vector_map),
      incbi(g, v_start, v_goal,
         start_predecessor, start_distance, start_distance_lookahead,
         goal_predecessor, goal_distance, goal_distance_lookahead,
         w_map,
         get(boost::vertex_index, g), // vertex_index_map
         lazysp_incsp_incbi_edge_index_adaptor<Graph,EdgeIndexMap>(g, edge_index_map),
         std::less<weight_type>(), // compare
         boost::closed_plus<weight_type>(std::numeric_limits<weight_type>::max()), // combine
         std::numeric_limits<weight_type>::max(),
         weight_type(),
         goal_margin,
         vis, incbi_balancer_distance<Vertex,weight_type>())
   {
   }
   
   weight_type solve(const Graph & g, Vertex v_start, Vertex v_goal,
      WMap wmap, std::vector<Edge> & path)
   {
      std::pair<size_t,bool> spresult = incbi.compute_shortest_path();
      
      // no solution?
      if (!spresult.second)
         return std::numeric_limits<weight_type>::max();
      
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
   
   void update_notify(Edge e)
   {
      Vertex va = source(e,g);
      Vertex vb = target(e,g);
      incbi.start_update_vertex(va);
      incbi.start_update_vertex(vb);
      incbi.goal_update_vertex(va);
      incbi.goal_update_vertex(vb);
      incbi.update_edge(e);
      incbi.update_edge(edge(target(e,g),source(e,g),g).first);
   }
};

template <class Graph, class WMap,
   class StartPredecessorMap, class StartDistanceMap, class StartDistanceLookaheadMap,
   class GoalPredecessorMap, class GoalDistanceMap, class GoalDistanceLookaheadMap,
   class EdgeIndexMap, class EdgeVectorMap, class IncBiVisitor>
lazysp_incsp_incbi<Graph,WMap,StartPredecessorMap,StartDistanceMap,StartDistanceLookaheadMap,GoalPredecessorMap,GoalDistanceMap,GoalDistanceLookaheadMap,EdgeIndexMap,EdgeVectorMap,IncBiVisitor>
make_lazysp_incsp_incbi(
   Graph & g,
   typename boost::graph_traits<Graph>::vertex_descriptor v_start,
   typename boost::graph_traits<Graph>::vertex_descriptor v_goal,
   WMap w_map,
   StartPredecessorMap start_predecessor,
   StartDistanceMap start_distance, StartDistanceLookaheadMap start_distance_lookahead,
   GoalPredecessorMap goal_predecessor,
   GoalDistanceMap goal_distance, GoalDistanceLookaheadMap goal_distance_lookahead,
   EdgeIndexMap edge_index_map, EdgeVectorMap edge_vector_map,
   typename boost::property_traits<WMap>::value_type goal_margin,
   IncBiVisitor vis)
{
   return lazysp_incsp_incbi<Graph,WMap,StartPredecessorMap,StartDistanceMap,StartDistanceLookaheadMap,GoalPredecessorMap,GoalDistanceMap,GoalDistanceLookaheadMap,EdgeIndexMap,EdgeVectorMap,IncBiVisitor>(
      g, v_start, v_goal, w_map, start_predecessor, start_distance, start_distance_lookahead, goal_predecessor, goal_distance, goal_distance_lookahead, edge_index_map, edge_vector_map, goal_margin, vis);
}

} // namespace pr_bgl
