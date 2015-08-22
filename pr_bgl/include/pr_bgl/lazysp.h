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
   class EvalStrategy, class LazySPVisitor>
bool lazy_shortest_path(Graph & g,
   typename boost::graph_traits<Graph>::vertex_descriptor v_start,
   typename boost::graph_traits<Graph>::vertex_descriptor v_goal,
   WMap wmap, WLazyMap wlazymap, IsEvaledMap isevaledmap,
   std::vector<typename boost::graph_traits<Graph>::edge_descriptor> & path,
   EvalStrategy evalstrategy, LazySPVisitor visitor)
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;

   for (;;)
   {
      // find the lazy shortest path, for now using dijkstra's
      // TODO: we should use faster temporary storage for these!
      std::map<Vertex,Vertex> startpreds;
      std::map<Vertex,double> startdist;
      boost::dijkstra_shortest_paths(
         g,
         v_start,
         boost::make_assoc_property_map(startpreds),
         boost::make_assoc_property_map(startdist),
         wlazymap,
         boost::get(boost::vertex_index, g), // implicit vertex index map
         std::less<double>(), // compare
         boost::closed_plus<double>(std::numeric_limits<double>::max()), // combine
         std::numeric_limits<double>::max(),
         double(),
         boost::make_dijkstra_visitor(boost::null_visitor())
      );
      
      if (startdist[v_goal] == std::numeric_limits<double>::max())
      {
         visitor.no_path();
         return false;
      }
      
      // get path
      std::vector<Vertex> vpath;
      std::vector< std::pair<Edge,bool> > eepath;
      Vertex v_walk = v_goal;
      bool path_evaled = true;
      for (;;)
      {
         vpath.push_back(v_walk);
         if (v_walk == v_start)
            break;
         Vertex v_pred = startpreds[v_walk];
         std::pair<Edge,bool> ret = boost::edge(v_pred, v_walk, g);
         BOOST_ASSERT(ret.second);
         bool is_evaled = get(isevaledmap,ret.first);
         if (!is_evaled)
            path_evaled = false;
         eepath.push_back(std::make_pair(ret.first, is_evaled));
         v_walk = v_pred;
      }
      std::reverse(vpath.begin(),vpath.end());
      std::reverse(eepath.begin(),eepath.end());
      
      visitor.lazy_path(startdist[v_goal], vpath);
      
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
      for (unsigned int ui=0; ui<to_evaluate.size(); ui++)
      {
         Edge & e = to_evaluate[ui];
         visitor.edge_evaluate(e);
         put(wlazymap, e, get(wmap,e));
         put(isevaledmap, e, true);
      }
   }
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
      for (i=0; i<path.size(); i++)
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
         else if (i==path.size()-1)
            rev_dist = 1;
         else
            rev_dist = dists[i+1] + 1;
         if (rev_dist < dists[i])
            dists[i] = rev_dist;
      }
      // choose max
      int max = -1;
      int max_i = 0;
      for (i=0; i<path.size(); i++)
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
};
         

} // namespace pr_bgl
