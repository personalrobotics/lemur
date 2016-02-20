/*! \file partition_simple.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Function (pr_bgl::partition_simple) for calculate the
 *        edge-weight partition function over all simple paths between
 *        every pair of vertices on a graph.
 */

 
namespace pr_bgl
{

template <class Graph>
struct partition_simple_el
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   Vertex v; // source vertex
   double len; // total len from start
   std::pair<EdgeOutIter,EdgeOutIter> es;
   double score; // total score through this vertex
   partition_simple_el(Vertex v, double len, std::pair<EdgeOutIter,EdgeOutIter> es):
      v(v), len(len), es(es), score(0) {}
};

/*!
 * The partition_simple function calculates an approximation to the
 * edge-weight partition function over all simple paths between two
 * given vertices. The implementation returns the value over all simple
 * paths with total length below a given parameter.
 * 
\verbatim
WeightMap: edges to weights (double?)
DistanceMap: vertices to distances
ScoreMap: edges to scores
\endverbatim
 */
template <class Graph, class WeightMap, class DistanceMap, class ScoreMap, class IsUsedMap>
double partition_simple(
   const Graph & g,
   typename boost::graph_traits<Graph>::vertex_descriptor v_start,
   typename boost::graph_traits<Graph>::vertex_descriptor v_goal,
   double beta,
   double len_max,
   WeightMap weight_map,
   DistanceMap goal_distance_map,
   ScoreMap score_map,
   IsUsedMap is_used_map)
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::graph_traits<Graph>::edge_iterator EdgeIter;
   
   // clear edge scores
   for (std::pair<EdgeIter,EdgeIter> ep=edges(g); ep.first!=ep.second; ep.first++)
      put(score_map, *ep.first, 0.0);
   
   // track total score
   double score_total = 0.0;
   
   // if true, the vertex is used already (the source of an edge in the stack)
   for (std::pair<VertexIter,VertexIter> vp=vertices(g); vp.first!=vp.second; vp.first++)
      put(is_used_map, *vp.first, false);
   
   put(is_used_map, v_start, true);
   std::vector< partition_simple_el<Graph> > stack;
   stack.push_back(partition_simple_el<Graph>(v_start, 0.0, out_edges(v_start, g)));
   
   while (stack.size())
   {
      // are we done?
      if (stack.back().es.first == stack.back().es.second)
      {
         put(is_used_map, stack.back().v, false);
         // back-copy score
         double score_popped = stack.back().score;
         stack.pop_back();
         if (stack.size())
         {
            put(score_map, *stack.back().es.first,
               get(score_map, *stack.back().es.first) + score_popped);
            
            stack.back().es.first++;
            stack.back().score += score_popped;
         }
         else
         {
            score_total = score_popped;
         }
         continue;
      }
      
      // ok, the last edge iterator points to a new edge;
      // look at the vertex at the end
      Vertex v_next = target(*stack.back().es.first, g);
      
      // is this vertex already visited?
      if (get(is_used_map, v_next))
      {
         stack.back().es.first++;
         continue;
      }
      double len = stack.back().len + get(weight_map, *stack.back().es.first);
      
      // is the path so far to this target next vertex too long?
      if (len_max < len + get(goal_distance_map, v_next) )
      {
         stack.back().es.first++;
         continue;
      }
      
      // is this the goal vertex?
      if (v_next == v_goal)
      {
         // compute score for this path
         double score = exp(-beta*len);
         
         stack.back().score += score;
         
         // add to last edge
         put(score_map, *stack.back().es.first,
            get(score_map, *stack.back().es.first) + score);
         
         stack.back().es.first++;
         continue;
      }
      
      // ok, we reached a new vertex!
      put(is_used_map, v_next, true);
      stack.push_back(partition_simple_el<Graph>(v_next, len, out_edges(v_next, g)));
   }
   
   return score_total;
}

} // namespace pr_bgl
