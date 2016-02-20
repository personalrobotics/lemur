/*! \file lazysp.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Lazy shortest path search (pr_bgl::lazysp).
 */

namespace pr_bgl
{

/*! \brief Invoke the Lazy Shortest Path graph search algorithm
 *
 * The lazysp function implements the Lazy Shortest Path algorithm for
 * the single-pair shortest path problem. It takes as an argument an
 * EvalStrategy object which determins for the candidate path found at
 * each iteration which edge(s) to select for evaluation.
 * 
 * Related code:
 *
 * - lazysp_incsp_astar.h - adaptor to use A* for inner search
 * - lazysp_incsp_dijkstra.h - adaptor to use Dijkstra's algorithm for
 *   inner search
 * - lazysp_incsp_incbi.h - adaptor to use incremental bidirectional
 *   algorithm for inner search
 * - lazysp_incsp_lpastar.h - adaptor to use LPA* for inner
 *   search
 * - lazysp_selector_partition_all.h - selector using partition
 *   functions
 * - lazysp_selector_sp_indicator_probability.h - selector using sp
 *   indicator probability
 * 
 * WMap is NOT assumed to be cached
 * (i.e. it's ok if it's expensive to evaluate each time)
 */
template <class Graph,
   class WMap, class WLazyMap, class IsEvaledMap,
   class IncSP, class EvalStrategy, class LazySPVisitor>
bool lazysp(Graph & g,
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
      visitor.selector_begin();
      evalstrategy.get_to_evaluate(g, eepath, to_evaluate);
      visitor.selector_end();
      BOOST_ASSERT(to_evaluate.size());

      // perform the evaluations
      
      for (unsigned int ui=0; ui<to_evaluate.size(); ui++)
      {
         Edge & e = to_evaluate[ui];
         weight_type e_weight_old = get(wlazymap, e);
         
         visitor.eval_begin();
         weight_type e_weight = get(wmap,e);
         visitor.eval_end();
         
         visitor.edge_evaluate(e, e_weight);
         put(wlazymap, e, e_weight);
         
         incsp.update_notify(e);
         
         visitor.selector_notify_begin();
         evalstrategy.update_notify(e, e_weight_old);
         visitor.selector_notify_end();
      }
      
   }
}

/*! \brief Null visitor for pr_bgl::lazysp.
 */
class lazysp_visitor_null
{
public:
   
   lazysp_visitor_null() {}
   
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
   
   inline void selector_begin() {}
   inline void selector_end() {}
   
   inline void selector_notify_begin() {}
   inline void selector_notify_end() {}
};


/*! \brief Pair visitor for pr_bgl::lazysp.
 */
template <class A, class B>
class lazysp_visitor_pair
{
public:
   A visA;
   B visB;
   lazysp_visitor_pair(A visA, B visB): visA(visA), visB(visB) {}
   
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
   
   inline void selector_begin()
   {
      visA.selector_begin();
      visB.selector_begin();
   }
   
   inline void selector_end()
   {
      visA.selector_end();
      visB.selector_end();
   }
   
   inline void selector_notify_begin()
   {
      visA.selector_notify_begin();
      visB.selector_notify_begin();
   }
   
   inline void selector_notify_end()
   {
      visA.selector_notify_end();
      visB.selector_notify_end();
   }
   
};

template <class A, class B>
lazysp_visitor_pair<A,B>
make_lazysp_visitor_pair(A visA, B visB)
{
   return lazysp_visitor_pair<A,B>(visA, visB);
}

/*! \brief Forward selector for pr_bgl::lazysp.
 */
class lazysp_selector_fwd
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

/*! \brief Reverse selector for pr_bgl::lazysp.
 */
class lazysp_selector_rev
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

/*! \brief Alternate selector for pr_bgl::lazysp.
 */
class lazysp_selector_alt
{
public:
   lazysp_selector_fwd fwd;
   lazysp_selector_rev rev;
   bool do_fwd;
   lazysp_selector_alt(): do_fwd(true) {}
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

/*! \brief Bisect selector for pr_bgl::lazysp.
 */
class lazysp_selector_bisect
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

/*! \brief FwdExpand selector for pr_bgl::lazysp.
 */
class lazysp_selector_fwdexpand
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

} // namespace pr_bgl
