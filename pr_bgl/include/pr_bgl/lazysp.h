/* File: lazysp.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace pr_bgl
{

// lazy shortest path algorithm
// NOT ompl-specific
//
// eventually this should also be templated on
// the inner SP algorithm type
//
// WMap is NOT assumed to be cached
// (i.e. it's ok if it's expensive to evaluate each time)
//
// also, should this be a class?
// (do we construct anything each iteration?)
template <class Graph,
   class WMap, class WLazyMap, class IsEvaledMap,
   class IncSP, class EvalStrategy, class LazySPVisitor>
bool lazy_shortest_path(Graph & g,
   typename boost::graph_traits<Graph>::vertex_descriptor v_start,
   typename boost::graph_traits<Graph>::vertex_descriptor v_goal,
   WMap wmap, WLazyMap wlazymap, IsEvaledMap isevaledmap,
   std::vector<typename boost::graph_traits<Graph>::edge_descriptor> & path,
   IncSP incsp, EvalStrategy evalstrategy, LazySPVisitor visitor)
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef typename boost::property_traits<WMap>::value_type weight_type;

   for (;;)
   {
      std::vector<Edge> incsp_path;
      
      visitor.search_begin();
      weight_type pathlen = incsp.solve(g, v_start, v_goal, wlazymap, incsp_path);
      visitor.search_end();
      
      if (pathlen == std::numeric_limits<weight_type>::max())
      {
         visitor.no_path();
         return false;
      }
      
      // compose vpath and eepath, determine if path already evaled
      std::vector<Vertex> vpath;
      std::vector< std::pair<Edge,bool> > eepath;
      bool path_evaled = true;
      vpath.push_back(source(incsp_path[0],g));
      for (unsigned int ui=0; ui<incsp_path.size(); ui++)
      {
         Edge & e = incsp_path[ui];
         vpath.push_back(target(e,g));
         bool is_evaled = get(isevaledmap, e);
         if (!is_evaled)
            path_evaled = false;
         eepath.push_back(std::make_pair(e, is_evaled));
      }
      
      visitor.lazy_path(pathlen, vpath, eepath);
      
      if (path_evaled)
      {
         visitor.path_found();
         for (unsigned int ui=0; ui<eepath.size(); ui++)
            path.push_back(eepath[ui].first);
         return true;
      }

      // determine edges to evaluate
      std::vector<Edge> to_evaluate;
      evalstrategy.get_to_evaluate(g, eepath, to_evaluate);
      BOOST_ASSERT(to_evaluate.size());

      // perform the evaluations
      visitor.eval_begin();
      for (unsigned int ui=0; ui<to_evaluate.size(); ui++)
      {
         Edge & e = to_evaluate[ui];
         weight_type e_weight_old = get(wlazymap, e);
         weight_type e_weight = get(wmap,e);
         visitor.edge_evaluate(e, e_weight);
         put(wlazymap, e, e_weight);
         incsp.update_notify(e);
         evalstrategy.update_notify(e, e_weight_old);
      }
      visitor.eval_end();
   }
}

class lazysp_null_visitor
{
public:
   
   lazysp_null_visitor() {}
   
   template <class Vertex, class Edge>
   inline void lazy_path(double length,
      std::vector<Vertex> & vpath,
      std::vector< std::pair<Edge,bool> > & eepath)
   {
   }

   inline void search_begin() {}
   inline void search_end() {}
   
   inline void eval_begin() {}
   inline void eval_end() {}

   inline void no_path() {}
   inline void path_found() {}
   
   template <class Edge>
   inline void edge_evaluate(Edge & e, double e_weight) {}
};

template <class A, class B>
class lazysp_null_visitor_pair
{
public:
   A visA;
   B visB;
   lazysp_null_visitor_pair(A visA, B visB): visA(visA), visB(visB) {}
   
   template <class Vertex, class Edge>
   inline void lazy_path(double length,
      std::vector<Vertex> & vpath,
      std::vector< std::pair<Edge,bool> > & eepath)
   {
      visA.lazy_path(length, vpath, eepath);
      visB.lazy_path(length, vpath, eepath);
   }

   inline void search_begin()
   {
      visA.search_begin();
      visB.search_begin();
   }
   
   inline void search_end()
   {
      visA.search_end();
      visB.search_end();
   }
   
   inline void eval_begin()
   {
      visA.eval_begin();
      visB.eval_begin();
   }
   
   inline void eval_end()
   {
      visA.eval_end();
      visB.eval_end();
   }

   inline void no_path()
   {
      visA.no_path();
      visB.no_path();
   }
   
   inline void path_found()
   {
      visA.path_found();
      visB.path_found();
   }
   
   template <class Edge>
   inline void edge_evaluate(Edge & e, double e_weight)
   {
      visA.edge_evaluate(e, e_weight);
      visB.edge_evaluate(e, e_weight);
   }
};

template <class A, class B>
lazysp_null_visitor_pair<A,B> make_lazysp_null_visitor_pair(A visA, B visB)
{
   return lazysp_null_visitor_pair<A,B>(visA, visB);
}


class LazySpEvalFwd
{
public:
   template <class Graph>
   void get_to_evaluate(
      const Graph & g,
      const std::vector< std::pair<typename boost::graph_traits<Graph>::edge_descriptor,bool> > & path,
      std::vector<typename boost::graph_traits<Graph>::edge_descriptor> & to_evaluate)
   {
      unsigned int ui;
      for (ui=0; ui<path.size(); ui++)
         if (path[ui].second == false)
            break;
      if (ui<path.size())
         to_evaluate.push_back(path[ui].first);
   }
   template <class Edge, class WeightType>
   void update_notify(Edge e, WeightType e_weight_old) {}
};

class LazySpEvalRev
{
public:
   template <class Graph>
   void get_to_evaluate(
      const Graph & g,
      const std::vector< std::pair<typename boost::graph_traits<Graph>::edge_descriptor,bool> > & path,
      std::vector<typename boost::graph_traits<Graph>::edge_descriptor> & to_evaluate)
   {
      int i;
      for (i=path.size()-1; 0<=i; i--)
         if (path[i].second == false)
            break;
      if (0<=i)
         to_evaluate.push_back(path[i].first);
   }
   template <class Edge, class WeightType>
   void update_notify(Edge e, WeightType e_weight_old) {}
};

class LazySpEvalAlt
{
public:
   LazySpEvalFwd fwd;
   LazySpEvalRev rev;
   bool do_fwd;
   LazySpEvalAlt(): do_fwd(true) {}
   template <class Graph>
   void get_to_evaluate(
      const Graph & g,
      const std::vector< std::pair<typename boost::graph_traits<Graph>::edge_descriptor,bool> > & path,
      std::vector<typename boost::graph_traits<Graph>::edge_descriptor> & to_evaluate)
   {
      if (do_fwd)
         fwd.get_to_evaluate(g, path, to_evaluate);
      else
         rev.get_to_evaluate(g, path, to_evaluate);
      do_fwd = !do_fwd;
   }
   template <class Edge, class WeightType>
   void update_notify(Edge e, WeightType e_weight_old) {}
};

class LazySpEvalBisect
{
public:
   template <class Graph>
   void get_to_evaluate(
      const Graph & g,
      const std::vector< std::pair<typename boost::graph_traits<Graph>::edge_descriptor,bool> > & path,
      std::vector<typename boost::graph_traits<Graph>::edge_descriptor> & to_evaluate)
   {
      // dists = min(dist_backwards, dist_forwards)
      // distance for now is just number of edges
      // dist[e_evaled] = 0
      int i;
      std::vector<int> dists(path.size());
      // forward
      for (i=0; i<(int)path.size(); i++)
      {
         int fwd_dist;
         if (path[i].second)
            fwd_dist = 0;
         else if (i==0)
            fwd_dist = 1;
         else
            fwd_dist = dists[i-1] + 1;
         dists[i] = fwd_dist;
      }
      // backwards
      for (i=path.size()-1; i>=0; i--)
      {
         int rev_dist;
         if (path[i].second)
            rev_dist = 0;
         else if (i==(int)path.size()-1)
            rev_dist = 1;
         else
            rev_dist = dists[i+1] + 1;
         if (rev_dist < dists[i])
            dists[i] = rev_dist;
      }
      // choose max
      int max = -1;
      int max_i = 0;
      for (i=0; i<(int)path.size(); i++)
      {
         if (max < dists[i])
         {
            max = dists[i];
            max_i = i;
         }
      }
      if (max < 0)
      {
         printf("error with max!\n");
         abort();
      }
      to_evaluate.push_back(path[max_i].first);
   }
   template <class Edge, class WeightType>
   void update_notify(Edge e, WeightType e_weight_old) {}
};

class LazySpEvalFwdExpand
{
public:
   template <class Graph>
   void get_to_evaluate(
      const Graph & g,
      const std::vector< std::pair<typename boost::graph_traits<Graph>::edge_descriptor,bool> > & path,
      std::vector<typename boost::graph_traits<Graph>::edge_descriptor> & to_evaluate)
   {
      unsigned int ui;
      for (ui=0; ui<path.size(); ui++)
         if (path[ui].second == false)
            break;
      if (!(ui<path.size()))
         return;
      typename boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
      for (boost::tie(ei,ei_end)=out_edges(source(path[ui].first,g),g); ei!=ei_end; ++ei)
         to_evaluate.push_back(*ei);
   }
   template <class Edge, class WeightType>
   void update_notify(Edge e, WeightType e_weight_old) {}
};

// solve returns weight_type::max if a non-infinite path is found
// solve is always called with the same g,v_start,v_goal
template <class Graph, class WMap>
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
   
   lazysp_incsp_dijkstra() {}
   
   weight_type solve(const Graph & g, Vertex v_start, Vertex v_goal,
      WMap wmap, std::vector<Edge> & path)
   {
      std::map<Vertex,Vertex> startpreds;
      std::map<Vertex,weight_type> startdist;
      try
      {
         boost::dijkstra_shortest_paths(
            g,
            v_start,
            boost::make_assoc_property_map(startpreds),
            boost::make_assoc_property_map(startdist),
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
         
      if (startdist[v_goal] == std::numeric_limits<weight_type>::max())
         return std::numeric_limits<weight_type>::max();
      
      // get path
      path.clear();
      for (Vertex v_walk=v_goal; v_walk!=v_start;)
      {
         Vertex v_pred = startpreds[v_walk];
         std::pair<Edge,bool> ret = edge(v_pred, v_walk, g);
         BOOST_ASSERT(ret.second);
         path.push_back(ret.first);
         v_walk = v_pred;
      }
      std::reverse(path.begin(),path.end());
      
      return startdist[v_goal];
   }
   
   void update_notify(Edge e)
   {
   }
};

// solve returns weight_type::max if a non-infinite path is found
// solve is always called with the same g,v_start,v_goal
template <class Graph, class WMap, class HeuristicMap, class CostMap, class ColorMap>
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
   CostMap cost_map;
   ColorMap color_map;
   lazysp_incsp_astar(HeuristicMap heuristic_map, CostMap cost_map, ColorMap color_map):
      heuristic_map(heuristic_map), cost_map(cost_map), color_map(color_map)
   {}
   
   weight_type solve(const Graph & g, Vertex v_start, Vertex v_goal,
      WMap wmap, std::vector<Edge> & path)
   {
      std::map<Vertex,Vertex> startpreds;
      std::map<Vertex,weight_type> startdist;
      try
      {
         astar_search(
            g,
            v_start,
            map_heuristic(heuristic_map), // AStarHeuristic h
            throw_visitor(v_goal), // AStarVisitor vis
            boost::make_assoc_property_map(startpreds), // PredecessorMap predecessor
            cost_map, // CostMap cost
            boost::make_assoc_property_map(startdist), // DistanceMap distance
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
         
      if (startdist[v_goal] == std::numeric_limits<weight_type>::max())
         return std::numeric_limits<weight_type>::max();
      
      // get path
      path.clear();
      for (Vertex v_walk=v_goal; v_walk!=v_start;)
      {
         Vertex v_pred = startpreds[v_walk];
         std::pair<Edge,bool> ret = edge(v_pred, v_walk, g);
         BOOST_ASSERT(ret.second);
         path.push_back(ret.first);
         v_walk = v_pred;
      }
      std::reverse(path.begin(),path.end());
      
      return startdist[v_goal];
   }
   
   void update_notify(Edge e)
   {
   }
};

template <class Graph, class WMap, class HeuristicMap, class CostMap, class ColorMap>
lazysp_incsp_astar<Graph,WMap,HeuristicMap,CostMap,ColorMap>
make_lazysp_incsp_astar(HeuristicMap heuristic_map, CostMap cost_map, ColorMap color_map)
{
   return lazysp_incsp_astar<Graph,WMap,HeuristicMap,CostMap,ColorMap>(heuristic_map, cost_map, color_map);
}

} // namespace pr_bgl
