/* File: graph_io.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace pr_bgl
{

template <class Graph, class VertexIndexMap, class EdgeIndexMap, class EdgeVectorMap>
class GraphIO
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   
public:
   Graph & g;
   VertexIndexMap vertex_index_map;
   EdgeIndexMap edge_index_map;
   EdgeVectorMap edge_vector_map;
   
   GraphIO(Graph & g, VertexIndexMap vertex_index_map, EdgeIndexMap edge_index_map, EdgeVectorMap edge_vector_map):
      g(g), vertex_index_map(vertex_index_map), edge_index_map(edge_index_map), edge_vector_map(edge_vector_map)
   {}
   
   // dump a graph
   void dump_graph(std::ostream & os)
   {
      // write all vertices
      typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
      for (boost::tie(vi,vi_end)=boost::vertices(g); vi!=vi_end; ++vi)
      {
         std::size_t index = get(vertex_index_map, *vi);
         os << "vertex " << index << "\n";
      }
         
      // write all edges
      typename boost::graph_traits<Graph>::edge_iterator ei, ei_end;
      for (boost::tie(ei,ei_end)=boost::edges(g); ei!=ei_end; ++ei)
      {
         std::size_t index = get(edge_index_map, *ei);
         os << "edge " << index
            << " source " << boost::source(*ei, g)
            << " target " << boost::target(*ei, g)
            << "\n";
      }
   }
   
   
   template <class Key>
   class IOBase
   {
   public:
      virtual void serialize(Key & e, std::ostream & os) = 0;
      virtual void deserialize(Key & e, std::istream & is) = 0;
   };

   typedef boost::shared_ptr< IOBase< Vertex > > IOBaseVertex;
   typedef boost::shared_ptr< IOBase< Edge > > IOBaseEdge;
   std::map<std::string, IOBaseVertex> vertex_serializers;
   std::map<std::string, IOBaseEdge> edge_serializers;
   
   
   
   
   template <class PropMap>
   class IOPropMap : public IOBase< typename PropMap::key_type >
   {
   public:
      PropMap prop_map;
      IOPropMap(PropMap prop_map): prop_map(prop_map) {}
      void serialize(typename PropMap::key_type & k, std::ostream & os)
      {
         os << get(prop_map, k);
      }
      void deserialize(typename PropMap::key_type & k, std::istream & is)
      {
         std::string tag;
         is >> tag; // GET EVERYTHING, SPACES INCLUDED?
         put(prop_map, k, tag);
      }
   };
   
   template <class PropMap>
   void add_property_map(const std::string & id, PropMap prop_map)
   {
      add_property_map(id, prop_map, (typename PropMap::key_type *)0);
   }
   
   template <class PropMap>
   void add_property_map(const std::string & id, PropMap prop_map, Vertex * v)
   {
      vertex_serializers[id] = IOBaseVertex(new IOPropMap<PropMap>(prop_map));
   }
   
   template <class PropMap>
   void add_property_map(const std::string & id, PropMap prop_map, Edge * e)
   {
      edge_serializers[id] = IOBaseEdge(new IOPropMap<PropMap>(prop_map));
   }
   
   void dump_properties(std::ostream & os)
   {
      typename std::map<std::string, IOBaseVertex>::iterator vser;
      for (vser=vertex_serializers.begin(); vser!=vertex_serializers.end(); vser++)
      {
         typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
         for (boost::tie(vi,vi_end)=boost::vertices(g); vi!=vi_end; ++vi)
         {
            typename boost::graph_traits<Graph>::vertex_descriptor v = *vi;
            std::size_t index = get(vertex_index_map, v);
            os << "vertex " << index
               << " " << vser->first << " ";
            vser->second->serialize(v, os);
            os << "\n";
         }
      }
      typename std::map<std::string, IOBaseEdge>::iterator eser;
      for (eser=edge_serializers.begin(); eser!=edge_serializers.end(); eser++)
      {
         typename boost::graph_traits<Graph>::edge_iterator ei, ei_end;
         for (boost::tie(ei,ei_end)=boost::edges(g); ei!=ei_end; ++ei)
         {
            typename boost::graph_traits<Graph>::edge_descriptor e = *ei;
            std::size_t index = get(edge_index_map, e);
            os << "edge " << index
               << " " << eser->first << " ";
            eser->second->serialize(e, os);
            os << "\n";
         }
      }
   }
   
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
         if (tag == "edge")
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
};

} // namespace pr_bgl
