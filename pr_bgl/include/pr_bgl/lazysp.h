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
      evalstrategy.get_to_evaluate(eepath, to_evaluate);
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
   template <class Edge>
   void get_to_evaluate(
      const std::vector< std::pair<Edge,bool> > & path,
      std::vector<Edge> & to_evaluate)
   {
      unsigned int ui;
      for (ui=0; ui<path.size(); ui++)
         if (path[ui].second == false)
            break;
      if (ui<path.size())
         to_evaluate.push_back(path[ui].first);
   }
};

} // namespace pr_bgl
