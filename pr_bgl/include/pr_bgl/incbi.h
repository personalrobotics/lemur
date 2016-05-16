/*! \file incbi.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Incremental bidirectional Dijkstra's search (pr_bgl::incbi).
 */

namespace pr_bgl
{

/*! \brief This class implements incremental bidirectional Dijkstra's
 *         search for the single-pair shortest path problem.
 * 
 * for now, we assume an undirected graph
 * (that is, update_edge will attempt an upate in both directions)
 * we assume that everything is constant (graph structure)
 * rhs = one-step-lookahead (based on d/g)
 * d (DynamicSWSF-FP) = g (LPA*) value = distance map
 * rhs (DynamicSWSF-FP) = rhs (LPA*) = distance_lookahead_map
 *
 * see "Efficient Point-to-Point Shortest Path Algorithms"
 * by Andrew V. Goldberg et. al
 * for correct bidirection Dijkstra's termination condition
 * 
 * original implemetation from 2015-04
 * 
 * be careful: this class uses the CostInf template argument for
 * decisions about infinities and non-existant paths;
 * if the underlying edge weights use a different value,
 * performace can suffer!
 * 
 * Invariant 1:
 * start_distlook[v] = min_pred(v) { start_dist[u] + w(u,v) }
 * 
 * Invariant 2:
 * iff start_distlook[v] != start_dist[u], then inconsistent
 * (also conn queue stuff)
 */
template <class Graph,
   class StartPredecessorMap,
   class StartDistanceMap, class StartDistanceLookaheadMap,
   class GoalSuccessorMap,
   class GoalDistanceMap, class GoalDistanceLookaheadMap,
   class WeightMap,
   class VertexIndexMap, class EdgeIndexMap,
   typename CompareFunction, typename CombineFunction,
   typename CostInf, typename CostZero,
   class IncBiVisitor, class IncBiBalancer>
class incbi
{
public:
   
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;
   typedef typename boost::graph_traits<Graph>::in_edge_iterator InEdgeIter;
   typedef typename boost::property_traits<WeightMap>::value_type weight_type;
   
   struct conn_key
   {
      weight_type path_length;
      weight_type start_dist;
      weight_type goal_dist;
      conn_key():
         path_length(weight_type()), start_dist(weight_type()), goal_dist(weight_type())
      {
      }
      conn_key(weight_type path_length, weight_type start_dist, weight_type goal_dist):
         path_length(path_length), start_dist(start_dist), goal_dist(goal_dist)
      {
      }
      bool operator<(const conn_key & rhs) const
      {
         return path_length < rhs.path_length;
      }
      bool operator>(const conn_key & rhs) const
      {
         return path_length > rhs.path_length;
      }
      bool operator<=(const conn_key & rhs) const
      {
         return path_length <= rhs.path_length;
      }
   };
   
   const Graph & g;
   Vertex v_start;
   Vertex v_goal;
   StartPredecessorMap start_predecessor;
   StartDistanceMap start_distance;
   StartDistanceLookaheadMap start_distance_lookahead;
   GoalSuccessorMap goal_successor;
   GoalDistanceMap goal_distance;
   GoalDistanceLookaheadMap goal_distance_lookahead;
   WeightMap weight;
   VertexIndexMap vertex_index_map;
   EdgeIndexMap edge_index_map;
   CompareFunction compare;
   CombineFunction combine;
   CostInf inf;
   CostZero zero;
   weight_type goal_margin;
   IncBiVisitor vis;
   IncBiBalancer balancer;
   
   // these contain all inconsistent vertices
   heap_indexed< weight_type > start_queue;
   heap_indexed< weight_type > goal_queue;
   
   // contains the indices of all edges connecting one start-tree vertex to one goal-tree vertex
   // that are both consistent, sorted by start_distance + edge_weight + goal_distance
   // infinite-length prospective paths are not in queue at all
   heap_indexed< conn_key > conn_queue;
   
   incbi(
      const Graph & g,
      Vertex v_start, Vertex v_goal,
      StartPredecessorMap start_predecessor,
      StartDistanceMap start_distance, StartDistanceLookaheadMap start_distance_lookahead,
      GoalSuccessorMap goal_successor,
      GoalDistanceMap goal_distance, GoalDistanceLookaheadMap goal_distance_lookahead,
      WeightMap weight,
      VertexIndexMap vertex_index_map,
      EdgeIndexMap edge_index_map,
      CompareFunction compare, CombineFunction combine,
      CostInf inf, CostZero zero,
      weight_type goal_margin,
      IncBiVisitor vis,
      IncBiBalancer balancer):
      g(g), v_start(v_start), v_goal(v_goal),
      start_predecessor(start_predecessor),
      start_distance(start_distance),
      start_distance_lookahead(start_distance_lookahead),
      goal_successor(goal_successor),
      goal_distance(goal_distance),
      goal_distance_lookahead(goal_distance_lookahead),
      weight(weight),
      vertex_index_map(vertex_index_map),
      edge_index_map(edge_index_map),
      compare(compare), combine(combine),
      inf(inf), zero(zero),
      goal_margin(goal_margin),
      vis(vis), balancer(balancer)
   {
      VertexIter vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      {
         put(start_distance_lookahead, *vi, inf);
         put(start_distance, *vi, inf);
         put(goal_distance_lookahead, *vi, inf);
         put(goal_distance, *vi, inf);
      }
      put(start_distance_lookahead, v_start, zero);
      put(goal_distance_lookahead, v_goal, zero);
      start_queue.insert(get(vertex_index_map,v_start), zero);
      vis.start_queue_insert(v_start);
      goal_queue.insert(get(vertex_index_map,v_goal), zero);
      vis.goal_queue_insert(v_goal);
   }
   
   inline weight_type start_calculate_key(Vertex u)
   {
      return std::min(get(start_distance,u), get(start_distance_lookahead,u));
   }
   
   inline weight_type goal_calculate_key(Vertex u)
   {
      return std::min(get(goal_distance,u), get(goal_distance_lookahead,u));
   }
   
   // this must be called when an edge's cost changes
   // in case it should be added/removed from the connection queue
   void update_edge(Edge e)
   {
      // should this edge be our connection queue?
      size_t eidx = get(edge_index_map, e);
      weight_type elen = get(weight, e);
      Vertex va = source(e,g);
      Vertex vb = target(e,g);
      // should it be in the queue?
      bool is_valid = false;
      do
      {
         if (elen == inf) break;
         if (start_queue.contains(get(vertex_index_map,va))) break;
         if (goal_queue.contains(get(vertex_index_map,vb))) break;
         if (get(start_distance,va) == inf) break;
         if (get(goal_distance,vb) == inf) break;
         is_valid = true;
      }
      while (0);
      // should we remove it?
      if (!is_valid)
      {
         if (conn_queue.contains(eidx))
         {
            conn_queue.remove(eidx);
            vis.conn_queue_remove(e);
         }
      }
      else
      {
         // ok, edge should be there!
         weight_type pathlen = combine(combine(get(start_distance,va), elen), get(goal_distance,vb));
         conn_key new_key(pathlen, get(start_distance,va), get(goal_distance,vb));
         if (conn_queue.contains(eidx))
         {
            conn_queue.update(eidx, new_key);
            vis.conn_queue_update(e);
         }
         else
         {
            conn_queue.insert(eidx, new_key);
            vis.conn_queue_insert(e);
         }
      }
   }
   
   // this is called to update vertex v due to either:
   // - u_dist being updated OR
   // - uv_weight being updated
   // it will recalculate v's lookahead distance
   // and return whether v's lookahead distance changed
   // (if predecessor changed, but lookahead value didnt, return false)
   inline bool start_update_predecessor(Vertex u, Vertex v, double uv_weight)
   {
      // start vertex dist lookahead is always zero
      if (v == v_start)
         return false;
      
      // current predecessor and lookahead value
      Vertex v_pred = get(start_predecessor, v);
      weight_type v_look_old = get(start_distance_lookahead, v);
      weight_type v_look_u = combine(get(start_distance,u), uv_weight);
      
      if (v_pred == u) // u was previously relied upon
      {
         // if dist through u decreased, then u is still best; just update value
         if (v_look_u == v_look_old)
         {
            return false;
         }
         else if (v_look_u < v_look_old)
         {
            put(start_distance_lookahead, v, v_look_u);
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
               weight_type v_look_uu = combine(get(start_distance,source(*ei,g)), get(weight,*ei));
               if (v_look_uu < v_look_best)
               {
                  v_look_best = v_look_uu;
                  v_pred_best = source(*ei,g);
               }
            }
            if (v_look_best != inf)
               put(start_predecessor, v, v_pred_best);
            if (v_look_best == v_look_old)
            {
               return false;
            }
            else
            {
               put(start_distance_lookahead, v, v_look_best);
               return true;
            }
         }
      }
      else // some other (existing) predecessor was used by v
      {
         if (v_look_u < v_look_old) // dist through u is better
         {
            put(start_predecessor, v, u);
            put(start_distance_lookahead, v, v_look_u);
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
   // this ensures u is in the right queues
   inline void start_update_vertex(Vertex u)
   {
      weight_type u_dist = get(start_distance,u);
      bool is_consistent = (u_dist == get(start_distance_lookahead,u));
      size_t u_idx = get(vertex_index_map,u);
      if (is_consistent)
      {
         if (start_queue.contains(u_idx))
         {
            start_queue.remove(u_idx);
            vis.start_queue_remove(u);
            // we're newly consistent, so insert any new conn_queue edges from us
            if (u_dist != inf)
            {
               OutEdgeIter ei, ei_end;
               for (boost::tie(ei,ei_end)=out_edges(u,g); ei!=ei_end; ei++)
               {
                  Vertex v = target(*ei,g);
                  size_t v_idx = get(vertex_index_map, v);
                  
                  weight_type v_goaldist = get(goal_distance,v);
                  if (!goal_queue.contains(v_idx) && v_goaldist != inf && get(weight,*ei) != inf)
                  {
                     conn_key new_key(combine(combine(u_dist, get(weight,*ei)), v_goaldist),
                        u_dist, v_goaldist);
                     conn_queue.insert(get(edge_index_map,*ei), new_key);
                     vis.conn_queue_insert(*ei);
                  }
               }
            }
         }
      }
      else // not consistent
      {
         if (start_queue.contains(u_idx))
         {
            start_queue.update(u_idx, start_calculate_key(u));
            vis.start_queue_update(u);
         }
         else
         {
            start_queue.insert(u_idx, start_calculate_key(u));
            vis.start_queue_insert(u);
            // we're newly inconsistent, so remove any conn_queue edges from us
            OutEdgeIter ei, ei_end;
            for (boost::tie(ei,ei_end)=out_edges(u,g); ei!=ei_end; ei++)
            {
               size_t edge_index = get(edge_index_map,*ei);
               if (conn_queue.contains(edge_index))
               {
                  conn_queue.remove(edge_index);
                  vis.conn_queue_remove(*ei);
               }
            }
         }
      }
   }
   
   // this is called to update vertex u due to either:
   // - v_dist being updated OR
   // - uv_weight being updated
   // it will recalculate u's lookahead distance
   // and return whether u's lookahead distance changed
   // (if predecessor changed, but lookahead value didnt, return false)
   inline bool goal_update_successor(Vertex u, Vertex v, double uv_weight)
   {
      // goal vertex dist lookahead is always zero
      if (u == v_goal)
         return false;
      
      // current successor and lookahead value
      Vertex u_succ = get(goal_successor, u);
      weight_type u_look_old = get(goal_distance_lookahead, u);
      weight_type u_look_v = combine(uv_weight, get(goal_distance,v));
      
      if (u_succ == v) // v was previously relied upon
      {
         // if dist through v decreased, then v is still best; just update value
         if (u_look_v == u_look_old)
         {
            return false;
         }
         else if (u_look_v < u_look_old)
         {
            put(goal_distance_lookahead, u, u_look_v);
            return true;
         }
         else // dist through v increased
         {
            // so we need to search for a potentially new best successor
            weight_type u_look_best = inf;
            Vertex u_succ_best;
            OutEdgeIter ei, ei_end;
            for (boost::tie(ei,ei_end)=out_edges(u,g); ei!=ei_end; ei++)
            {
               weight_type u_look_vv = combine(get(weight,*ei), get(goal_distance,target(*ei,g)));
               if (u_look_vv < u_look_best)
               {
                  u_look_best = u_look_vv;
                  u_succ_best = target(*ei,g);
               }
            }
            if (u_look_best != inf)
               put(goal_successor, u, u_succ_best);
            if (u_look_best == u_look_old)
            {
               return false;
            }
            else
            {
               put(goal_distance_lookahead, u, u_look_best);
               return true;
            }
         }
      }
      else // some other (existing) successor was used by u
      {
         if (u_look_v < u_look_old) // dist through v is better
         {
            put(goal_successor, u, v);
            put(goal_distance_lookahead, u, u_look_v);
            return true;
         }
         else // v is not better
         {
            return false;
         }
      }
   }
   
   // this must be called called when u's dist and/or lookahead dist
   // (and therefore consistency) may have been changed
   // this ensures v is in the right queues
   inline void goal_update_vertex(Vertex v)
   {
      weight_type v_dist = get(goal_distance,v);
      bool is_consistent = (v_dist == get(goal_distance_lookahead,v));
      size_t v_idx = get(vertex_index_map,v);
      if (is_consistent)
      {
         if (goal_queue.contains(v_idx))
         {
            goal_queue.remove(v_idx);
            vis.goal_queue_remove(v);
            // we're newly consistent, so insert any new conn_queue edges to us
            if (v_dist != inf)
            {
               InEdgeIter ei, ei_end;
               for (boost::tie(ei,ei_end)=in_edges(v,g); ei!=ei_end; ei++)
               {
                  Vertex u = source(*ei,g);
                  size_t u_idx = get(vertex_index_map, u);
                  
                  weight_type u_startdist = get(start_distance,u);
                  if (!start_queue.contains(u_idx) && u_startdist != inf && get(weight,*ei) != inf)
                  {
                     conn_key new_key(combine(combine(u_startdist, get(weight,*ei)), v_dist),
                        u_startdist, v_dist);
                     conn_queue.insert(get(edge_index_map,*ei), new_key);
                     vis.conn_queue_insert(*ei);
                  }
               }
            }
         }
      }
      else // not consistent
      {
         if (goal_queue.contains(v_idx))
         {
            goal_queue.update(v_idx, goal_calculate_key(v));
            vis.goal_queue_update(v);
         }
         else
         {
            goal_queue.insert(v_idx, goal_calculate_key(v));
            vis.goal_queue_insert(v);
            // we're newly inconsistent, so remove any conn_queue edges to us
            InEdgeIter ei, ei_end;
            for (boost::tie(ei,ei_end)=in_edges(v,g); ei!=ei_end; ei++)
            {
               size_t edge_index = get(edge_index_map,*ei);
               if (conn_queue.contains(edge_index))
               {
                  conn_queue.remove(edge_index);
                  vis.conn_queue_remove(*ei);
               }
            }
         }
      }
   }
   
   // returns index of middle edge, and bool for success
   std::pair<size_t,bool> compute_shortest_path()
   {
      for (;;)
      {
         weight_type start_top = start_queue.size() ? start_queue.top_key() : inf;
         weight_type goal_top = goal_queue.size() ? goal_queue.top_key() : inf;
         
         // no-path termination
         if (!start_queue.size() && !goal_queue.size())
            return std::make_pair(0, false);
         if (!start_queue.size() && get(start_distance,v_goal) == inf)
            return std::make_pair(0, false);
         if (!goal_queue.size() && get(goal_distance,v_start) == inf)
            return std::make_pair(0, false);
         
         // has-path termination condition is rather complicated!
         do
         {
            if (!conn_queue.size()) break;
            if (combine(conn_queue.top_key().path_length,goal_margin) > combine(start_top,goal_top)) break;
            if (combine(conn_queue.top_key().start_dist,goal_margin) > start_top) break;
            if (combine(conn_queue.top_key().goal_dist,goal_margin) > goal_top) break;
            return std::make_pair(conn_queue.top_idx(), true);
         }
         while (0);
         
         bool do_goal =
            start_queue.contains(v_start) ? false :
            goal_queue.contains(v_goal) ? true :
            balancer(start_top,goal_top,start_queue.size(),goal_queue.size());
         
         if (do_goal == false)
         {
            if (!start_queue.size())
               return std::make_pair(0, false);
            
            size_t u_idx = start_queue.top_idx();
            Vertex u = vertex(u_idx, g);
            
            vis.examine_vertex_start(u);
            
            start_queue.remove_min();
            vis.start_queue_remove(u);
            if (get(start_distance,u) > get(start_distance_lookahead,u))
            {
               weight_type u_sdist = get(start_distance_lookahead,u);
               put(start_distance, u, u_sdist);
               
               // vertex u is newly start-consistent
               
               // update any successors that they may now be inconsistent
               // also, this start vertex just became consistent,
               // so add any out_edges to consistent goal-tree vertices
               // to conn queue
               OutEdgeIter ei, ei_end;
               for (boost::tie(ei,ei_end)=out_edges(u,g); ei!=ei_end; ei++)
               {
                  Vertex v = target(*ei,g);
                  size_t v_idx = get(vertex_index_map, v);
                  
                  double uv_weight = get(weight,*ei);
                  bool lookahead_changed = start_update_predecessor(u, v, uv_weight);
                  if (lookahead_changed)
                     start_update_vertex(v);
                  
                  weight_type v_tdist = get(goal_distance,v);
                  if (u_sdist != inf && !goal_queue.contains(v_idx) && v_tdist != inf && uv_weight != inf)
                  {
                     conn_key new_key(combine(combine(u_sdist, uv_weight), v_tdist), u_sdist, v_tdist);
                     conn_queue.insert(get(edge_index_map,*ei), new_key);
                     vis.conn_queue_insert(*ei);
                  }
               }
            }
            else
            {
               put(start_distance, u, inf);
               start_update_vertex(u);
               OutEdgeIter ei, ei_end;
               for (boost::tie(ei,ei_end)=out_edges(u,g); ei!=ei_end; ei++)
               {
                  Vertex v = target(*ei,g);
                  
                  double uv_weight = get(weight,*ei);
                  bool lookahead_changed = start_update_predecessor(u, v, uv_weight);
                  if (lookahead_changed)
                     start_update_vertex(v);
               }
            }
         }
         else
         {
            if (!goal_queue.size())
               return std::make_pair(0, false);
            
            size_t v_idx = goal_queue.top_idx();
            Vertex v = vertex(v_idx, g);
            
            vis.examine_vertex_goal(v);
            
            goal_queue.remove_min();
            vis.goal_queue_remove(v);
            if (get(goal_distance,v) > get(goal_distance_lookahead,v))
            {
               weight_type v_tdist = get(goal_distance_lookahead,v);
               put(goal_distance, v, v_tdist);
               
               // vertex v is newly goal-consistent
               
               // update any predecessors that they may now be inconsistent
               // also, this goal vertex just became consistent,
               // so add any in_edges from consistent start-tree vertices
               InEdgeIter ei, ei_end;
               for (boost::tie(ei,ei_end)=in_edges(v,g); ei!=ei_end; ei++)
               {
                  Vertex u = source(*ei,g);
                  size_t u_idx = get(vertex_index_map, u);
                  
                  double uv_weight = get(weight,*ei);
                  bool lookahead_changed = goal_update_successor(u, v, uv_weight);
                  if (lookahead_changed)
                     goal_update_vertex(u);
                  
                  weight_type u_sdist = get(start_distance,u);
                  if (v_tdist != inf && !start_queue.contains(u_idx) && u_sdist != inf && uv_weight != inf)
                  {
                     conn_key new_key(combine(combine(u_sdist, uv_weight), v_tdist), u_sdist, v_tdist);
                     conn_queue.insert(get(edge_index_map,*ei), new_key);
                     vis.conn_queue_insert(*ei);
                  }
               }
            }
            else
            {
               put(goal_distance, v, inf);
               goal_update_vertex(v);
               InEdgeIter ei, ei_end;
               for (boost::tie(ei,ei_end)=in_edges(v,g); ei!=ei_end; ei++)
               {
                  Vertex u = source(*ei,g);
                  
                  double uv_weight_new = get(weight,*ei);
                  bool lookahead_changed = goal_update_successor(u, v, uv_weight_new);
                  if (lookahead_changed)
                     goal_update_vertex(u);
               }
            }
         }
      }
   }

};

template <class Graph>
class incbi_visitor_null
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   inline void examine_vertex_start(Vertex v) {}
   inline void examine_vertex_goal(Vertex v) {}
   inline void start_queue_insert(Vertex v) {}
   inline void start_queue_update(Vertex v) {}
   inline void start_queue_remove(Vertex v) {}
   inline void goal_queue_insert(Vertex v) {}
   inline void goal_queue_update(Vertex v) {}
   inline void goal_queue_remove(Vertex v) {}
   inline void conn_queue_insert(Edge e) {}
   inline void conn_queue_update(Edge e) {}
   inline void conn_queue_remove(Edge e) {}
};

/*! \brief Balance expansions to the side with the shortest distance.
 * 
 * true = expand from goal side */
template <typename Vertex, typename weight_type>
struct incbi_balancer_distance
{
   const double goalfrac;
   incbi_balancer_distance(double goalfrac): goalfrac(goalfrac) {}
   bool operator()(
      weight_type start_top, weight_type goal_top,
      size_t start_queuesize, size_t goal_queuesize) const
   {
      return ((1.0-goalfrac)*goal_top < (goalfrac)*start_top);
   }
};

/*! \brief Balance expansions to the side with the smallest OPEN set
 *         cardinality.
 * 
 * true = expand from goal side */
template <typename Vertex, typename weight_type>
struct incbi_balancer_cardinality
{
   const double goalfrac;
   incbi_balancer_cardinality(double goalfrac): goalfrac(goalfrac) {}
   bool operator()(
      weight_type start_top, weight_type goal_top,
      size_t start_queuesize, size_t goal_queuesize) const
   {
      return ((1.0-goalfrac)*goal_queuesize < (goalfrac)*start_queuesize);
   }
};

} // namespace pr_bgl
