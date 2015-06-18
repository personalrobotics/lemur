
namespace pr_bgl
{

template <class Graph>
struct soft_edge_bc_el
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   Vertex v;
   double len;
   std::pair<EdgeOutIter,EdgeOutIter> es;
   //int es_num;
   soft_edge_bc_el(Vertex v, double len, std::pair<EdgeOutIter,EdgeOutIter> es):
      v(v), len(len), es(es)/*, es_num(0)*/ {}
};

template <class Graph, class WeightMap, class DistanceMap, class ScoreMap>
void soft_edge_bc(
   const Graph & g,
   typename boost::graph_traits<Graph>::vertex_descriptor v_start,
   typename boost::graph_traits<Graph>::vertex_descriptor v_goal,
   boost::function<double (double)> f_importance,
   double len_max,
   WeightMap weight_map,
   DistanceMap goal_distance_map,
   ScoreMap score_map)
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::graph_traits<Graph>::edge_iterator EdgeIter;
   
   // clear edge scores
   for (std::pair<EdgeIter,EdgeIter> ep=boost::edges(g); ep.first!=ep.second; ep.first++)
      boost::put(score_map, *ep.first, 0.0);
   
   // track total score
   double score_total = 0.0;
   
   // if true, the vertex is used already (the source of an edge in the stack)
   std::map<Vertex,bool> v_used;
   for (std::pair<VertexIter,VertexIter> vp=boost::vertices(g); vp.first!=vp.second; vp.first++)
      v_used[*vp.first] = false;
   
   v_used[v_start] = true;
   std::vector< soft_edge_bc_el<Graph> > stack;
   stack.push_back(soft_edge_bc_el<Graph>(v_start, 0.0, boost::out_edges(v_start, g)));
   
   while (stack.size())
   {
      // are we done?
      if (stack.back().es.first == stack.back().es.second)
      {
         v_used[stack.back().v] = false;
         stack.pop_back();
         stack.back().es.first++;
         //stack.back().es_num++;
         //if (stack.size() <= 3)
         //{
         //   printf("stack iters:");
         //   for (unsigned int ui=0; ui<stack.size(); ui++)
         //      printf(" %i", stack[ui].es_num);
         //   printf("\n");
         //}
         continue;
      }
      
      // ok, the last edge iterator points to a new edge;
      // look at the vertex at the end
      Vertex v_next = boost::target(*stack.back().es.first, g);
      
      // is this vertex already visited?
      if (v_used[v_next])
      {
         stack.back().es.first++;
         //stack.back().es_num++;
         continue;
      }
      
      double len = stack.back().len + boost::get(weight_map, *stack.back().es.first);
      
      // is this path so far to this vertex too long?
      if (len_max < len + boost::get(goal_distance_map, v_next) )
      {
         stack.back().es.first++;
         //stack.back().es_num++;
         continue;
      }
      
      // is this the goal vertex?
      if (v_next == v_goal)
      {
         //printf("found a path with len %f!\n", len);
         
         // compute score for this path
         double score = f_importance(len);
         
         for (int i=0; i<stack.size(); i++)
            boost::put(score_map, *stack[i].es.first,
               boost::get(score_map, *stack[i].es.first) + score);
         
         score_total += score;
         
         stack.back().es.first++;
         //stack.back().es_num++;
         continue;
      }
      
      // ok, we reached a new vertex!
      v_used[v_next] = true;
      stack.push_back(soft_edge_bc_el<Graph>(v_next, len, boost::out_edges(v_next, g)));
   }
   
   for (std::pair<EdgeIter,EdgeIter> ep=boost::edges(g); ep.first!=ep.second; ep.first++)
      boost::put(score_map, *ep.first,
         boost::get(score_map, *ep.first) / score_total);
}

} // namespace pr_bgl
