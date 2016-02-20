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
 */
template <class Graph,
   class StartPredecessorMap,
   class StartDistanceMap, class StartDistanceLookaheadMap,
   class GoalPredecessorMap,
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
   GoalPredecessorMap goal_predecessor;
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
   
   heap_indexed< weight_type > start_queue;
   heap_indexed< weight_type > goal_queue;
   
   // contains the indices of all edges connecting one start-tree vertex to one goal-tree vertex
   // that are both consistent, sorted by start_distance + edge_weight + goal_distance
   heap_indexed< conn_key > conn_queue;
   
   incbi(
      const Graph & g,
      Vertex v_start, Vertex v_goal,
      StartPredecessorMap start_predecessor,
      StartDistanceMap start_distance, StartDistanceLookaheadMap start_distance_lookahead,
      GoalPredecessorMap goal_predecessor,
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
      goal_predecessor(goal_predecessor),
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
   
   inline void start_update_vertex(Vertex u)
   {
      size_t u_idx = get(vertex_index_map,u);
      // when update is called on the start vertex itself,
      // don't touch it's lookahead distance -- it should stay 0.0
      if (u != v_start)
      {
         weight_type rhs = inf;
         InEdgeIter ei, ei_end;
         for (boost::tie(ei,ei_end)=in_edges(u,g); ei!=ei_end; ei++)
         {
            weight_type val = combine(get(start_distance,source(*ei,g)), get(weight,*ei));
            if (val < rhs)
            {
               rhs = val;
               put(start_predecessor, u, source(*ei,g));
            }
         }
         put(start_distance_lookahead, u, rhs);
      }
      weight_type u_dist = get(start_distance,u);
      bool is_consistent = (u_dist == get(start_distance_lookahead,u));
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
                  Vertex v_target = target(*ei,g);
                  size_t idx_target = get(vertex_index_map, v_target);
                  
                  weight_type goaldist_target = get(goal_distance,v_target);
                  if (!goal_queue.contains(idx_target) && goaldist_target != inf && get(weight,*ei) != inf)
                  {
                     conn_key new_key(combine(combine(u_dist, get(weight,*ei)), goaldist_target),
                        u_dist, goaldist_target);
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
   
   inline void goal_update_vertex(Vertex u)
   {
      size_t u_idx = get(vertex_index_map,u);
      // when update is called on the goal vertex itself,
      // don't touch it's lookahead distance -- it should stay 0.0
      if (u != v_goal)
      {
         weight_type rhs = inf;
         OutEdgeIter ei, ei_end;
         for (boost::tie(ei,ei_end)=out_edges(u,g); ei!=ei_end; ei++)
         {
            weight_type val = combine(get(weight,*ei), get(goal_distance,target(*ei,g)));
            if (val < rhs)
            {
               rhs = val;
               put(goal_predecessor, u, target(*ei,g));
            }
         }
         put(goal_distance_lookahead, u, rhs);
      }
      weight_type u_dist = get(goal_distance,u);
      bool is_consistent = (u_dist == get(goal_distance_lookahead,u));
      if (is_consistent)
      {
         if (goal_queue.contains(u_idx))
         {
            goal_queue.remove(u_idx);
            vis.goal_queue_remove(u);
            // we're newly consistent, so insert any new conn_queue edges to us
            if (u_dist != inf)
            {
               InEdgeIter ei, ei_end;
               for (boost::tie(ei,ei_end)=in_edges(u,g); ei!=ei_end; ei++)
               {
                  Vertex v_source = source(*ei,g);
                  size_t idx_source = get(vertex_index_map, v_source);
                  
                  weight_type startdist_source = get(start_distance,v_source);
                  if (!start_queue.contains(idx_source) && startdist_source != inf && get(weight,*ei) != inf)
                  {
                     conn_key new_key(combine(combine(startdist_source, get(weight,*ei)), u_dist),
                        startdist_source, u_dist);
                     conn_queue.insert(get(edge_index_map,*ei), new_key);
                     vis.conn_queue_insert(*ei);
                  }
               }
            }
         }
      }
      else // not consistent
      {
         if (goal_queue.contains(u_idx))
         {
            goal_queue.update(u_idx, goal_calculate_key(u));
            vis.goal_queue_update(u);
         }
         else
         {
            goal_queue.insert(u_idx, goal_calculate_key(u));
            vis.goal_queue_insert(u);
            // we're newly inconsistent, so remove any conn_queue edges to us
            InEdgeIter ei, ei_end;
            for (boost::tie(ei,ei_end)=in_edges(u,g); ei!=ei_end; ei++)
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
   
   // returns index of middle edge!
   std::pair<size_t,bool> compute_shortest_path()
   {
      for (;;)
      {
         weight_type start_top = start_queue.size() ? start_queue.top_key() : inf;
         weight_type goal_top = goal_queue.size() ? goal_queue.top_key() : inf;
         if (start_top == inf && goal_top == inf)
            return std::make_pair(0, false);
         
         // termination condition is rather complicated!
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
               weight_type u_dist = get(start_distance_lookahead,u);
               put(start_distance, u, u_dist);
               
               // update any successors that they may now be inconsistent
               // also, this start vertex just became consistent,
               // so add any out_edges to consistent goal-tree vertices
               OutEdgeIter ei, ei_end;
               for (boost::tie(ei,ei_end)=out_edges(u,g); ei!=ei_end; ei++)
               {
                  Vertex v_target = target(*ei,g);
                  size_t idx_target = get(vertex_index_map, v_target);
                  
                  start_update_vertex(v_target);
                  
                  weight_type goaldist_target = get(goal_distance,v_target);
                  if (u_dist != inf && !goal_queue.contains(idx_target) && goaldist_target != inf && get(weight,*ei) != inf)
                  {
                     conn_key new_key(combine(combine(u_dist, get(weight,*ei)), goaldist_target),
                        u_dist, goaldist_target);
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
                  start_update_vertex(target(*ei,g));
            }
         }
         else
         {
            if (!goal_queue.size())
               return std::make_pair(0, false);
            
            size_t u_idx = goal_queue.top_idx();
            Vertex u = vertex(u_idx, g);
            
            vis.examine_vertex_goal(u);
            
            goal_queue.remove_min();
            vis.goal_queue_remove(u);
            if (get(goal_distance,u) > get(goal_distance_lookahead,u))
            {
               weight_type u_dist = get(goal_distance_lookahead,u);
               put(goal_distance, u, u_dist);
               
               // update any predecessors that they may now be inconsistent
               // also, this goal vertex just became consistent,
               // so add any in_edges from consistent start-tree vertices
               InEdgeIter ei, ei_end;
               for (boost::tie(ei,ei_end)=in_edges(u,g); ei!=ei_end; ei++)
               {
                  Vertex v_source = source(*ei,g);
                  size_t idx_source = get(vertex_index_map, v_source);
                  
                  goal_update_vertex(v_source);
                  
                  weight_type startdist_source = get(start_distance,v_source);
                  if (u_dist != inf && !start_queue.contains(idx_source) && startdist_source != inf && get(weight,*ei) != inf)
                  {
                     conn_key new_key(combine(combine(startdist_source, get(weight,*ei)), u_dist),
                        startdist_source, u_dist);
                     conn_queue.insert(get(edge_index_map,*ei), new_key);
                     vis.conn_queue_insert(*ei);
                  }
               }
            }
            else
            {
               put(goal_distance, u, inf);
               goal_update_vertex(u);
               OutEdgeIter ei, ei_end;
               for (boost::tie(ei,ei_end)=out_edges(u,g); ei!=ei_end; ei++)
                  goal_update_vertex(target(*ei,g));
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
   bool operator()(
      weight_type start_top, weight_type goal_top,
      size_t start_queuesize, size_t goal_queuesize) const
   {
      return (goal_top < start_top);
   }
};

/*! \brief Balance expansions to the side with the smallest OPEN set
 *         cardinality.
 * 
 * true = expand from goal side */
template <typename Vertex, typename weight_type>
struct incbi_balancer_cardinality
{
   bool operator()(
      weight_type start_top, weight_type goal_top,
      size_t start_queuesize, size_t goal_queuesize) const
   {
      return (goal_queuesize < start_queuesize);
   }
};

} // namespace pr_bgl
