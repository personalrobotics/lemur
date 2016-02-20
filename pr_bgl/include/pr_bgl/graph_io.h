/*! \file graph_io.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Contains functions for working with graphio, a custom graph
 *        serialization format.
 */

namespace pr_bgl
{

/*! \brief Write a graph's structure to the custom graphio format.
 * 
 * This function implements serializing and deserializing a graph to a
 * custom textual graph format, which stores vertices,
 * edges and properties one per line.
 */
template<typename Graph, typename VertexIndexMap, typename EdgeIndexMap>
void
write_graphio_graph(std::ostream & out, const Graph & g,
   VertexIndexMap vertex_index_map, EdgeIndexMap edge_index_map)
{
   // write all vertices
   typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
   for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
   {
      std::size_t index = get(vertex_index_map, *vi);
      out << "vertex " << index << "\n";
   }
      
   // write all edges
   typename boost::graph_traits<Graph>::edge_iterator ei, ei_end;
   for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ++ei)
   {
      std::size_t index = get(edge_index_map, *ei);
      out << "edge " << index
         << " source " << boost::source(*ei, g)
         << " target " << boost::target(*ei, g)
         << "\n";
   }
}

/*! \brief Write a graph's properties to the custom graphio format.
 * 
 * This function implements serializing and deserializing a graph to a
 * custom textual graph format, which stores vertices,
 * edges and properties one per line.
 */
template<typename Graph, typename VertexIndexMap, typename EdgeIndexMap>
void
write_graphio_properties(std::ostream & out, const Graph & g,
   VertexIndexMap vertex_index_map, EdgeIndexMap edge_index_map,
   boost::dynamic_properties & properties)
{
   boost::dynamic_properties::iterator it;
   for (it=properties.begin(); it!=properties.end(); it++)
   {
      if (it->second->value() != typeid(std::string))
         throw std::runtime_error("a property map is not string-valued!");
      if (it->second->key() == typeid(typename boost::graph_traits<Graph>::vertex_descriptor))
      {
         typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
         for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
         {
            std::size_t index = get(vertex_index_map, *vi);
            std::string value = boost::any_cast<std::string>(it->second->get(*vi));
            out << "vprop " << index << " " << it->first << " " << value << "\n";
         }
      }
      else if (it->second->key() == typeid(typename boost::graph_traits<Graph>::edge_descriptor))
      {
         typename boost::graph_traits<Graph>::edge_iterator ei, ei_end;
         for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ++ei)
         {
            std::size_t index = get(edge_index_map, *ei);
            std::string value = boost::any_cast<std::string>(it->second->get(*ei));
            out << "eprop " << index << " " << it->first << " " << value << "\n";
         }
      }
      else
         throw std::runtime_error("a property map is not vertex-keyed or edge-keyed!");
   }
}

#if 0
void load_properties(std::istream & is)
{
   std::string str;
   while (std::getline(is,str))
   {
      std::stringstream ss;
      ss << str;
      // get line tag
      std::string tag;
      ss >> tag;
      if (tag == "eprop")
      {
         // get edge index
         std::size_t index;
         ss >> index;
         // get potential property name
         ss >> tag;
         typename std::map<std::string, IOBaseEdge>::iterator found = edge_serializers.find(tag);
         if (found == edge_serializers.end())
            continue;
         // delegate to iobase!
         found->second->deserialize(edge_vector_map[index], ss);
      }
   }
}
#endif

} // namespace pr_bgl
