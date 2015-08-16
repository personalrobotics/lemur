/* File: edge_indexed_graph.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace pr_bgl
{

template <class Graph, class EdgeIndexMap>
class EdgeIndexedGraph
{
public:
   typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
   typedef typename boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
   typedef typename boost::graph_traits<Graph>::adjacency_iterator adjacency_iterator;
   typedef typename boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
   typedef typename boost::graph_traits<Graph>::in_edge_iterator in_edge_iterator;
   typedef typename boost::graph_traits<Graph>::vertex_iterator vertex_iterator;
   typedef typename boost::graph_traits<Graph>::edge_iterator edge_iterator;
   typedef typename boost::graph_traits<Graph>::directed_category directed_category;
   typedef typename boost::graph_traits<Graph>::edge_parallel_category edge_parallel_category;
   typedef typename boost::graph_traits<Graph>::traversal_category traversal_category;
   typedef typename boost::graph_traits<Graph>::vertices_size_type vertices_size_type;
   typedef typename boost::graph_traits<Graph>::edges_size_type edges_size_type;
   typedef typename boost::graph_traits<Graph>::degree_size_type degree_size_type;

   Graph & m_g;
   EdgeIndexMap edge_index_map;
   boost::vector_property_map<edge_descriptor> edge_vector_map;
   
   EdgeIndexedGraph(Graph & g, EdgeIndexMap edge_index_map):
      m_g(g),
      edge_index_map(edge_index_map),
      edge_vector_map(0)
   {
      BOOST_ASSERT(num_edges(g) == 0);
   }
   
   static vertex_descriptor null_vertex()
   {
      return boost::graph_traits<Graph>::null_vertex();
   }
};

template <class Graph, class EdgeIndexMap>
inline typename boost::graph_traits<Graph>::vertices_size_type
num_vertices(const EdgeIndexedGraph<Graph,EdgeIndexMap> & g)
{
   return num_vertices(g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline typename boost::graph_traits<Graph>::edges_size_type
num_edges(const EdgeIndexedGraph<Graph,EdgeIndexMap> & g)
{
   return num_edges(g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline typename boost::graph_traits<Graph>::vertex_descriptor
vertex(typename boost::graph_traits<Graph>::vertices_size_type n, const EdgeIndexedGraph<Graph,EdgeIndexMap> & g)
{
   return vertex(n, g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline std::pair<typename boost::graph_traits<Graph>::vertex_iterator, typename boost::graph_traits<Graph>::vertex_iterator>
vertices(const EdgeIndexedGraph<Graph,EdgeIndexMap> & g)
{
   return vertices(g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline typename boost::graph_traits<Graph>::vertex_descriptor
add_vertex(EdgeIndexedGraph<Graph,EdgeIndexMap> & g)
{
   return add_vertex(g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline std::pair<typename boost::graph_traits<Graph>::edge_descriptor, bool>
add_edge(
   typename boost::graph_traits<Graph>::vertex_descriptor u,
   typename boost::graph_traits<Graph>::vertex_descriptor v,
   EdgeIndexedGraph<Graph,EdgeIndexMap> & g)
{
   std::pair<typename boost::graph_traits<Graph>::edge_descriptor, bool>
      res = add_edge(u, v, g.m_g);
   if (res.second)
   {
      put(g.edge_index_map, res.first, num_edges(g.m_g)-1);
      put(g.edge_vector_map, num_edges(g.m_g)-1, res.first);
   }
   return res;
}

template <class Graph, class EdgeIndexMap>
inline void
remove_vertex(
   typename boost::graph_traits<Graph>::vertex_descriptor u,
   EdgeIndexedGraph<Graph,EdgeIndexMap> & g)
{
   remove_vertex(u, g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline void
remove_edge(
   typename boost::graph_traits<Graph>::edge_descriptor e,
   EdgeIndexedGraph<Graph,EdgeIndexMap> & g)
{
   // ensure that we're removing edges in reverse order
   // (we may be able to loosen this requirement a bit!)
   BOOST_ASSERT(get(g.edge_index_map, e) == (num_edges(g)-1));
   // assume that external edge index map will get cleaned up for us
   // leave edge_vector_map alone
   // (indices bigger than num_edges()-1
   //  will just map to bogus edge descriptors)
   remove_edge(e, g.m_g);
}

} // namespace pr_bgl
