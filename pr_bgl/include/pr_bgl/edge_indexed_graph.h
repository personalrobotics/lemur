/*! \file edge_indexed_graph.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Contains pr_bgl::edge_indexed_graph.
 */

namespace pr_bgl
{

/*! \brief Graph wrapper which maintains an edge index.
 * 
 * The EdgeIndexedGraph class wraps an existing graph object, while
 * additionally maintaining incrementing edge indices in supplied
 * property maps. The behavior is undefined if edges are not removed
 * in the inverse order that they are added.
 */
template <class Graph, class EdgeIndexMap>
class edge_indexed_graph
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
   
   typedef typename boost::vector_property_map<edge_descriptor> EdgeVectorMap;

   Graph & m_g;
   size_t edge_count;
   EdgeIndexMap edge_index_map;
   EdgeVectorMap edge_vector_map;
   
   edge_indexed_graph(Graph & g, EdgeIndexMap edge_index_map):
      m_g(g),
      edge_count(0),
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
num_vertices(const edge_indexed_graph<Graph,EdgeIndexMap> & g)
{
   return num_vertices(g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline typename boost::graph_traits<Graph>::edges_size_type
num_edges(const edge_indexed_graph<Graph,EdgeIndexMap> & g)
{
   //return num_edges(g.m_g);
   return g.edge_count;
}

template <class Graph, class EdgeIndexMap>
inline typename boost::graph_traits<Graph>::vertex_descriptor
vertex(typename boost::graph_traits<Graph>::vertices_size_type n, const edge_indexed_graph<Graph,EdgeIndexMap> & g)
{
   return vertex(n, g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline std::pair<typename boost::graph_traits<Graph>::vertex_iterator, typename boost::graph_traits<Graph>::vertex_iterator>
vertices(const edge_indexed_graph<Graph,EdgeIndexMap> & g)
{
   return vertices(g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline std::pair<typename boost::graph_traits<Graph>::edge_iterator, typename boost::graph_traits<Graph>::edge_iterator>
edges(const edge_indexed_graph<Graph,EdgeIndexMap> & g)
{
   return edges(g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline typename boost::graph_traits<Graph>::vertex_descriptor
source(typename boost::graph_traits<Graph>::edge_descriptor e, const edge_indexed_graph<Graph,EdgeIndexMap> & g)
{
   return source(e, g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline typename boost::graph_traits<Graph>::vertex_descriptor
target(typename boost::graph_traits<Graph>::edge_descriptor e, const edge_indexed_graph<Graph,EdgeIndexMap> & g)
{
   return target(e, g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline typename boost::graph_traits<Graph>::vertex_descriptor
add_vertex(edge_indexed_graph<Graph,EdgeIndexMap> & g)
{
   return add_vertex(g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline std::pair<typename boost::graph_traits<Graph>::edge_descriptor, bool>
add_edge(
   typename boost::graph_traits<Graph>::vertex_descriptor u,
   typename boost::graph_traits<Graph>::vertex_descriptor v,
   edge_indexed_graph<Graph,EdgeIndexMap> & g)
{
   std::pair<typename boost::graph_traits<Graph>::edge_descriptor, bool>
      res = add_edge(u, v, g.m_g);
   if (res.second)
   {
      put(g.edge_index_map, res.first, g.edge_count);
      put(g.edge_vector_map, g.edge_count, res.first);
      g.edge_count++;
   }
   return res;
}

template <class Graph, class EdgeIndexMap>
inline void
remove_vertex(
   typename boost::graph_traits<Graph>::vertex_descriptor u,
   edge_indexed_graph<Graph,EdgeIndexMap> & g)
{
   remove_vertex(u, g.m_g);
}

template <class Graph, class EdgeIndexMap>
inline void
remove_edge(
   typename boost::graph_traits<Graph>::edge_descriptor e,
   edge_indexed_graph<Graph,EdgeIndexMap> & g)
{
   // ensure that we're removing edges in reverse order
   // (we may be able to loosen this requirement a bit!)
   BOOST_ASSERT(get(g.edge_index_map, e) == (g.edge_count-1));
   // assume that external edge index map will get cleaned up for us
   // leave edge_vector_map alone
   // (indices bigger than num_edges()-1
   //  will just map to bogus edge descriptors)
   remove_edge(e, g.m_g);
   g.edge_count--;
}

} // namespace pr_bgl
