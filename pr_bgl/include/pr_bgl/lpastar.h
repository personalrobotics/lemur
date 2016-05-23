/*! \file lpastar.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Lifelong Planning A* (pr_bgl::lpastar).
 */

namespace pr_bgl
{

/*! \brief Implements the Lifelong Planning A* incremental search
 *         algorithm.
 * 
 * Sven Koenig, Maxim Likhachev, and David Furcy. 2004.
 * Lifelong planning A*. Artif. Intell. 155, 1-2 (May 2004), 93-146.
 * DOI=http://dx.doi.org/10.1016/j.artint.2003.12.001
 * 
 * for now, we assume an undirected graph
 * (that is, update_edge will attempt an upate in both directions)
 * we assume that everything is constant (graph structure)
\verbatim
rhs = one-step-lookahead (based on d/g)
d (DynamicSWSF-FP) = g (LPA*) value = distance map
rhs (DynamicSWSF-FP) = rhs (LPA*) = distance_lookahead_map
\endverbatim
 */
template <typename Graph, typename AStarHeuristic,
   typename LPAStarVisitor, typename PredecessorMap,
   typename DistanceMap, typename DistanceLookaheadMap,
   typename WeightMap, typename VertexIndexMap,
   typename CompareFunction, typename CombineFunction,
   typename CostInf, typename CostZero>
class lpastar
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;
   typedef typename boost::graph_traits<Graph>::in_edge_iterator InEdgeIter;
   typedef typename boost::property_traits<WeightMap>::value_type weight_type;
   
   const Graph & g;
   Vertex v_start;
   Vertex v_goal;
   AStarHeuristic h;
   LPAStarVisitor vis;
   PredecessorMap predecessor;
   DistanceMap distance;
   DistanceLookaheadMap distance_lookahead;
   WeightMap weight;
   VertexIndexMap index_map;
   CompareFunction compare;
   CombineFunction combine;
   CostInf inf;
   CostZero zero;
   weight_type goal_margin;
   
   heap_indexed< std::pair<weight_type,weight_type> > queue;
   
   lpastar(
      const Graph & g,
      Vertex v_start, Vertex v_goal,
      AStarHeuristic h,
      LPAStarVisitor vis,
      PredecessorMap predecessor,
      DistanceMap distance, DistanceLookaheadMap distance_lookahead,
      WeightMap weight,
      VertexIndexMap index_map,
      CompareFunction compare, CombineFunction combine,
      CostInf inf, CostZero zero,
      weight_type goal_margin):
      g(g), v_start(v_start), v_goal(v_goal),
      h(h), vis(vis), predecessor(predecessor),
      distance(distance), distance_lookahead(distance_lookahead),
      weight(weight), index_map(index_map),
      compare(compare), combine(combine),
      inf(inf), zero(zero),
      goal_margin(goal_margin)
   {
      VertexIter vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      {
         put(distance_lookahead, *vi, inf);
         put(distance, *vi, inf);
      }
      put(predecessor, v_start, v_start);
      put(distance_lookahead, v_start, zero);
      queue.insert(get(index_map,v_start), std::make_pair(h(v_start),0));
   }
   
   inline std::pair<weight_type,weight_type> calculate_key(Vertex u, bool do_goal_margin=false)
   {
      weight_type minval
         = std::min(get(distance,u), get(distance_lookahead,u));
      if (do_goal_margin)
         minval += goal_margin;
      return std::make_pair(minval+h(u), minval);
   }
   
   // this is called to update vertex v due to either:
   // - u_dist being updated OR
   // - uv_weight being updated
   // it will recalculate v's lookahead distance
   // and return whether v's lookahead distance changed
   // (if predecessor changed, but lookahead value didnt, return false)
   inline bool update_predecessor(Vertex u, Vertex v, weight_type uv_weight)
   {
      // start vertex dist lookahead is always zero
      if (v == v_start)
         return false;
      
      // current predecessor and lookahead value
      Vertex v_pred = get(predecessor, v);
      weight_type v_look_old = get(distance_lookahead, v);
      weight_type v_look_u = combine(get(distance,u), uv_weight);
      
      if (v_pred == u) // u was previously relied upon
      {
         // if dist through u decreased, then u is still best; just update value
         if (v_look_u == v_look_old)
         {
            return false;
         }
         else if (v_look_u < v_look_old)
         {
            put(distance_lookahead, v, v_look_u);
            return true;
         }
         else // dist through u increased
         {
            // so we need to search for a potentially new best predecessessor
            weight_type v_look_best = inf;
            Vertex v_pred_best;
            InEdgeIter ei, ei_end;
            for (boost::tie(ei,ei_end)=in_edges(v,g); ei!=ei_end; ei++)
            {
               weight_type v_look_uu = combine(get(distance,source(*ei,g)), get(weight,*ei));
               if (v_look_uu < v_look_best)
               {
                  v_look_best = v_look_uu;
                  v_pred_best = source(*ei,g);
               }
            }
            if (v_look_best != inf)
               put(predecessor, v, v_pred_best);
            if (v_look_best == v_look_old)
            {
               return false;
            }
            else
            {
               put(distance_lookahead, v, v_look_best);
               return true;
            }
         }
      }
      else // some other (existing) predecessor was used by v
      {
         if (v_look_u < v_look_old) // dist through u is better
         {
            put(predecessor, v, u);
            put(distance_lookahead, v, v_look_u);
            return true;
         }
         else // u is not better
         {
            return false;
         }
      }
   }

   // this must be called called when u's dist and/or lookahead dist
   // (and therefore consistency) may have been changed
   // this ensures u is in the queue correctly
   inline void update_vertex(Vertex u)
   {
      weight_type u_dist = get(distance,u);
      bool is_consistent = (u_dist == get(distance_lookahead,u));
      size_t u_idx = get(index_map,u);
      if (is_consistent)
      {
         if (queue.contains(u_idx))
            queue.remove(u_idx);
      }
      else // not consistent
      {
         if (queue.contains(u_idx))
            queue.update(u_idx, calculate_key(u));
         else
            queue.insert(u_idx, calculate_key(u));
      }
   }

   void compute_shortest_path()
   {
      while (queue.size()
         && (queue.top_key() < calculate_key(v_goal,true) // do_goal_margin
         || get(distance_lookahead,v_goal) != get(distance,v_goal)))
      {
         Vertex u = vertex(queue.top_idx(), g);
         
         vis.examine_vertex(u, g);
         queue.remove_min();
         if (get(distance,u) > get(distance_lookahead,u))
         {
            put(distance, u, get(distance_lookahead,u));
            OutEdgeIter ei, ei_end;
            for (boost::tie(ei,ei_end)=out_edges(u,g); ei!=ei_end; ei++)
            {
               Vertex v = target(*ei,g);
               bool lookahead_changed = update_predecessor(u, v, get(weight,*ei));
               if (lookahead_changed)
                  update_vertex(v);
            }
         }
         else
         {
            put(distance, u, inf);
            update_vertex(u);
            OutEdgeIter ei, ei_end;
            for (boost::tie(ei,ei_end)=out_edges(u,g); ei!=ei_end; ei++)
            {
               Vertex v = target(*ei,g);
               bool lookahead_changed = update_predecessor(u, v, get(weight,*ei));
               if (lookahead_changed)
                  update_vertex(v);
            }
         }
      }
   }
};

} // namespace pr_bgl
