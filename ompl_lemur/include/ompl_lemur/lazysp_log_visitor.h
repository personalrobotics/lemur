/* File: lazysp_log_visitor.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

template <class VertexIndexMap, class EdgeIndexMap>
class LazySPLogVisitor : public pr_bgl::lazysp_visitor_null
{
public:
   
   VertexIndexMap vertex_index_map;
   EdgeIndexMap edge_index_map;
   std::ostream & stream;
   LazySPLogVisitor(
         VertexIndexMap vertex_index_map,
         EdgeIndexMap edge_index_map,
         std::ostream & stream):
      vertex_index_map(vertex_index_map),
      edge_index_map(edge_index_map),
      stream(stream)
   {
   }
   
   template <class Vertex, class Edge>
   inline void lazy_path(double length,
      std::vector<Vertex> & vpath,
      std::vector< std::pair<Edge,bool> > & eepath)
   {
      stream << "lazy_path_length " << length << std::endl;
      stream << "lazy_path";
      for (unsigned int ui=0; ui<vpath.size(); ui++)
         stream << " " << get(vertex_index_map,vpath[ui]);
      stream << std::endl;
      stream << "lazy_path_edge";
      for (unsigned int ui=0; ui<eepath.size(); ui++)
         stream << " " << get(edge_index_map,eepath[ui].first);
      stream << std::endl;
      stream << "lazy_path_edge_evaled";
      for (unsigned int ui=0; ui<eepath.size(); ui++)
         if (eepath[ui].second)
            stream << " true";
         else
            stream << " false";
      stream << std::endl;
   }

   inline void no_path()
   {
      stream << "no_path" << std::endl;
   }

   inline void path_found()
   {
      stream << "path_found" << std::endl;
   }
   
   template <class Edge>
   inline void edge_evaluate(Edge & e, double e_weight)
   {
      stream << "eval_edge " << get(edge_index_map,e)
         << " " << e_weight << std::endl;
   }
};

template <class VertexIndexMap, class EdgeIndexMap>
LazySPLogVisitor<VertexIndexMap,EdgeIndexMap>
make_lazysp_log_visitor(
   VertexIndexMap vertex_index_map, EdgeIndexMap edge_index_map, std::ostream & stream)
{
   return LazySPLogVisitor<VertexIndexMap,EdgeIndexMap>(
      vertex_index_map, edge_index_map, stream);
}




class LazySPTimingVisitor : public pr_bgl::lazysp_visitor_null
{
public:
   boost::chrono::high_resolution_clock::duration & dur_search;
   boost::chrono::high_resolution_clock::duration & dur_eval;
   boost::chrono::high_resolution_clock::duration & dur_selector;
   boost::chrono::high_resolution_clock::duration & dur_selector_notify;
   boost::chrono::high_resolution_clock::time_point time_begin;
   LazySPTimingVisitor(
      boost::chrono::high_resolution_clock::duration & dur_search,
      boost::chrono::high_resolution_clock::duration & dur_eval,
      boost::chrono::high_resolution_clock::duration & dur_selector,
      boost::chrono::high_resolution_clock::duration & dur_selector_notify):
      dur_search(dur_search), dur_eval(dur_eval),
      dur_selector(dur_selector), dur_selector_notify(dur_selector_notify)
   {
   }
   inline void search_begin()
   {
      time_begin = boost::chrono::high_resolution_clock::now();
   }
   inline void search_end()
   {
      dur_search += boost::chrono::high_resolution_clock::now() - time_begin;
   }
   inline void eval_begin()
   {
      time_begin = boost::chrono::high_resolution_clock::now();
   }
   inline void eval_end()
   {
      dur_eval += boost::chrono::high_resolution_clock::now() - time_begin;
   }
   
   inline void selector_begin()
   {
      time_begin = boost::chrono::high_resolution_clock::now();
   }
   
   inline void selector_end()
   {
      dur_selector += boost::chrono::high_resolution_clock::now() - time_begin;
   }
   
   inline void selector_notify_begin()
   {
      time_begin = boost::chrono::high_resolution_clock::now();
   }
   
   inline void selector_notify_end()
   {
      dur_selector_notify += boost::chrono::high_resolution_clock::now() - time_begin;
   }
};



} // namespace ompl_lemur
