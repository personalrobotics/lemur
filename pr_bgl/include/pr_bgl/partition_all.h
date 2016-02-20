/*! \file partition_all.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 * 
 * \brief Functions for calculate the edge-weight partition function
 *        over all paths between every pair of vertices on a graph.
 */

/* requires:
#include <boost/type_traits/is_base_and_derived.hpp>
#include <boost/type_traits/is_same.hpp>
*/

namespace pr_bgl
{

struct partition_all_divergent_exception {};

template <class Graph, class CouplingMap, class TempMap>
void partition_all_update_directed_edge(
   const Graph & g,
   typename boost::graph_traits<Graph>::vertex_descriptor v_a,
   typename boost::graph_traits<Graph>::vertex_descriptor v_b,
   double weight_frac,
   bool is_add,
   CouplingMap coupling_map,
   TempMap cs_xa, TempMap cs_by)
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::vertex_iterator VertexIter;
   double denom;
   if (is_add)
   {
      // update denominator
      denom = exp(weight_frac)
         - boost::get(coupling_map, std::make_pair(v_b,v_a));
      if (denom <= 0.0)
         throw partition_all_divergent_exception();
      //printf("adding with denom: %f\n", denom);
   }
   else
   {
      denom = - exp(weight_frac)
         - boost::get(coupling_map, std::make_pair(v_b,v_a));
   }
   // temporary storge of old c values
   for (std::pair<VertexIter,VertexIter> vp=boost::vertices(g);
         vp.first!=vp.second; vp.first++)
   {
      Vertex v = *vp.first;
      boost::put(cs_xa,v,
         boost::get(coupling_map, std::make_pair(v,v_a)));
      boost::put(cs_by,v,
         boost::get(coupling_map, std::make_pair(v_b,v)));
   }
   // iterate over all vertex pairs, updating coupling map
   for (std::pair<VertexIter,VertexIter> vpx=boost::vertices(g);
      vpx.first!=vpx.second; vpx.first++)
   {
      Vertex v_x = *vpx.first;
      for (std::pair<VertexIter,VertexIter> vpy=boost::vertices(g);
         vpy.first!=vpy.second; vpy.first++)
      {
         Vertex v_y = *vpy.first;
         boost::put(coupling_map, std::make_pair(v_x,v_y),
            boost::get(coupling_map, std::make_pair(v_x,v_y))
            + boost::get(cs_xa,v_x)*boost::get(cs_by,v_y)/denom);
      }
   }
}

template <class Graph, class CouplingMap, class TempMap>
void partition_all_update_edge(
   const Graph & g,
   typename boost::graph_traits<Graph>::edge_descriptor edge,
   double weight_frac,
   bool is_add,
   CouplingMap coupling_map,
   TempMap cs_xa, TempMap cs_by)
{
   typedef typename boost::graph_traits<Graph>::directed_category DirCat;
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   
   // taken from mesh_graph_generator.hpp
   BOOST_STATIC_CONSTANT(bool, is_undirected
      = (boost::is_base_and_derived<boost::undirected_tag, DirCat>::value
         || boost::is_same<boost::undirected_tag, DirCat>::value));
   
   Vertex v_s = boost::source(edge, g);
   Vertex v_t = boost::target(edge, g);
   
   // add forward edge s->t
   partition_all_update_directed_edge(g,
      v_s,
      v_t,
      weight_frac,
      is_add,
      coupling_map, cs_xa, cs_by);
   
   // add backwards edge t->s
   if (is_undirected)
   {
      partition_all_update_directed_edge(g,
         v_t,
         v_s,
         weight_frac,
         is_add,
         coupling_map, cs_xa, cs_by);
   }
}

template <class Graph, class CouplingMap>
void partition_all_init(
   const Graph & g,
   CouplingMap coupling_map)
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::vertex_iterator VertexIter;
   
   for (std::pair<VertexIter,VertexIter> vpx=boost::vertices(g);
      vpx.first!=vpx.second; vpx.first++)
   {
      Vertex v_x = *vpx.first;
      for (std::pair<VertexIter,VertexIter> vpy=boost::vertices(g);
         vpy.first!=vpy.second; vpy.first++)
      {
         Vertex v_y = *vpy.first;
         boost::put(coupling_map, std::make_pair(v_x,v_y),
            (v_x == v_y) ? 1.0 : 0.0);
      }
   }
}

/*! The partition_all functions calculate the edge-weight partition
 * function over all paths between every pair of vertices on a graph,
 * via a recursive formulation which is linear in the number of edges
 * in the graph. 
 * 
 * This function initializes the coupling_map, and goes over all edges.
 */
template <class Graph, class WeightMap, class CouplingMap, class TempMap>
void partition_all(
   const Graph & g,
   double len_ref,
   WeightMap weight_map,
   CouplingMap coupling_map,
   TempMap cs_xa, TempMap cs_by)
{
   typedef typename boost::graph_traits<Graph>::edge_iterator EdgeIter;
   
   partition_all_init(g, coupling_map);
   
   std::pair<EdgeIter,EdgeIter> ep=boost::edges(g);
   for (EdgeIter ei=ep.first; ei!=ep.second; ei++)
   {
      partition_all_update_edge(
         g,
         *ei,
         boost::get(weight_map,*ei) / len_ref,
         true, // is_add
         coupling_map, cs_xa, cs_by);
   }
}

template <class Graph, class CouplingMap>
double partition_all_without_edge(
   const Graph & g,
   typename boost::graph_traits<Graph>::vertex_descriptor v_x,
   typename boost::graph_traits<Graph>::vertex_descriptor v_y,
   typename boost::graph_traits<Graph>::edge_descriptor edge,
   double weight_frac,
   CouplingMap coupling_map)
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::directed_category DirCat;
   
   // taken from mesh_graph_generator.hpp
   BOOST_STATIC_CONSTANT(bool, is_undirected
      = (boost::is_base_and_derived<boost::undirected_tag, DirCat>::value
         || boost::is_same<boost::undirected_tag, DirCat>::value));
   
   Vertex v_s = boost::source(edge, g);
   Vertex v_t = boost::target(edge, g);
   
   if (is_undirected)
   {
      // lets do a two-step backup
      
      // original (double-prime) values
      double cpp_xy = boost::get(coupling_map, std::make_pair(v_x,v_y));
      double cpp_xs = boost::get(coupling_map, std::make_pair(v_x,v_s));
      double cpp_xt = boost::get(coupling_map, std::make_pair(v_x,v_t));
      double cpp_sy = boost::get(coupling_map, std::make_pair(v_s,v_y));
      double cpp_ss = boost::get(coupling_map, std::make_pair(v_s,v_s));
      double cpp_st = boost::get(coupling_map, std::make_pair(v_s,v_t));
      double cpp_ty = boost::get(coupling_map, std::make_pair(v_t,v_y));
      double cpp_ts = boost::get(coupling_map, std::make_pair(v_t,v_s));
      double cpp_tt = boost::get(coupling_map, std::make_pair(v_t,v_t));
      
      // first, remove the edge from s(a) --> t(b)
      // calculate some intermediate results we'll need later
      double cp_xy = cpp_xy - (cpp_xs * cpp_ty)/(exp(weight_frac) + cpp_ts);
      double cp_xt = cpp_xt - (cpp_xs * cpp_tt)/(exp(weight_frac) + cpp_ts);
      double cp_sy = cpp_sy - (cpp_ss * cpp_ty)/(exp(weight_frac) + cpp_ts);
      double cp_st = cpp_st - (cpp_ss * cpp_tt)/(exp(weight_frac) + cpp_ts);
      
      // next, remove the edge from b(s) --> a(t)
      double c_xy = cp_xy - (cp_xt * cp_sy)/(exp(weight_frac) + cp_st);
      return c_xy;
   }
   else
   {
      double cp_xy = boost::get(coupling_map, std::make_pair(v_x,v_y));
      double cp_xs = boost::get(coupling_map, std::make_pair(v_x,v_s));
      double cp_ty = boost::get(coupling_map, std::make_pair(v_t,v_y));
      double cp_ts = boost::get(coupling_map, std::make_pair(v_t,v_s));
      
      // remove the edge from s(a) --> t(b)
      double c_xy = cp_xy - (cp_xs * cp_ty)/(exp(weight_frac) + cp_ts);
      return c_xy;
   }
}

class partition_all_matrix
{
public:
   boost::numeric::ublas::matrix<double> Z;
   boost::numeric::ublas::vector<double> Z_xa;
   boost::numeric::ublas::vector<double> Z_by;

   partition_all_matrix(size_t num_vertices):
      Z(num_vertices,num_vertices),
      Z_xa(num_vertices), Z_by(num_vertices)
   {
      Z = boost::numeric::ublas::identity_matrix<double>(num_vertices);
   }
   
   inline void add_edge(size_t va, size_t vb, double weight_frac)
   {
      double denom = exp(weight_frac) - Z(vb, va);
      if (denom <= 0.0)
         throw std::runtime_error("divergent!");
      // copy in a-th column and b-th row
      Z_xa = column(Z, va);
      Z_by = row(Z, vb);
      // do outer product
      Z += (1.0/denom) * outer_prod(Z_xa, Z_by);
   }
   
   inline void remove_edge(size_t va, size_t vb, double weight_frac)
   {
      double denom = exp(weight_frac) + Z(vb, va);
      // copy in a-th column and b-th row
      Z_xa = column(Z, va);
      Z_by = row(Z, vb);
      // do outer product
      Z -= (1.0/denom) * outer_prod(Z_xa, Z_by);
   }
   
   inline double without_undirected(size_t vx, size_t vy, size_t va, size_t vb, double weight_frac)
   {
      // lets do a two-step backup

      // original (double-prime) values
      double cpp_xy = Z(vx, vy);
      double cpp_xs = Z(vx, va);
      double cpp_xt = Z(vx, vb);
      double cpp_sy = Z(va, vy);
      double cpp_ss = Z(va, va);
      double cpp_st = Z(va, vb);
      double cpp_ty = Z(vb, vy);
      double cpp_ts = Z(vb, va);
      double cpp_tt = Z(vb, vb);
      
      // first, remove the edge from s(a) --> t(b)
      // calculate some intermediate results we'll need later
      double cp_xy = cpp_xy - (cpp_xs * cpp_ty)/(exp(weight_frac) + cpp_ts);
      double cp_xt = cpp_xt - (cpp_xs * cpp_tt)/(exp(weight_frac) + cpp_ts);
      double cp_sy = cpp_sy - (cpp_ss * cpp_ty)/(exp(weight_frac) + cpp_ts);
      double cp_st = cpp_st - (cpp_ss * cpp_tt)/(exp(weight_frac) + cpp_ts);
      
      // next, remove the edge from b(s) --> a(t)
      double c_xy = cp_xy - (cp_xt * cp_sy)/(exp(weight_frac) + cp_st);
      return c_xy;
   }
};

} // namespace pr_bgl
