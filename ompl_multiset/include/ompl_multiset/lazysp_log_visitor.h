/* File: lazysp_log_visitor.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

template <class VertexIndexMap, class EdgeIndexMap>
class LazySPLogVisitor
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
   
   template <class Vertex>
   void lazy_path(double length, std::vector<Vertex> & vpath)
   {
      stream << "lazy_path_length " << length << std::endl;
      stream << "lazy_path";
      for (unsigned int ui=0; ui<vpath.size(); ui++)
         stream << " " << get(vertex_index_map,vpath[ui]);
      stream << std::endl;
   }

   void no_path()
   {
      stream << "no_path" << std::endl;
   }

   void path_found()
   {
      stream << "path_found" << std::endl;
   }
   
   template <class Edge>
   void edge_evaluate(Edge & e)
   {
      stream << "eval_edge " << get(edge_index_map,e) << std::endl;
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

} // namespace ompl_multiset
