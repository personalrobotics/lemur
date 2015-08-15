/* File: coupling.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

/* requires:
#include <boost/type_traits/is_base_and_derived.hpp>
#include <boost/type_traits/is_same.hpp>
*/

// WeightMap: edges to weights (double?)
// DistanceMap: vertices to distances
// ScoreMap: edges to scores

// turns out this is a vector outer product (x yT)!
// use a matrix library?

namespace pr_bgl
{

template <class Graph, class CouplingMap, class TempMap>
void coupling_update_directed_edge(
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
   if (!is_add)
   {
      printf("ERROR, INCREMENTAL REMOVE NOT YET IMPLEMENTED!\n");
      abort();
   }
   // update denominator
   double denom = exp(weight_frac)
      - boost::get(coupling_map, std::make_pair(v_b,v_a));
   if (denom <= 0.0)
   {
      printf("ERROR, SERIES DOES NOT CONVERGE!\n");
      abort();
   }
   //printf("adding with denom: %f\n", denom);
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
void coupling_update_edge(
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
   coupling_update_directed_edge(g,
      v_s,
      v_t,
      weight_frac,
      is_add,
      coupling_map, cs_xa, cs_by);
   
   // add backwards edge t->s
   if (is_undirected)
   {
      coupling_update_directed_edge(g,
         v_t,
         v_s,
         weight_frac,
         is_add,
         coupling_map, cs_xa, cs_by);
   }
}

template <class Graph, class CouplingMap>
void coupling_init(
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

// initializes the coupling_map, and goes over all edges
template <class Graph, class WeightMap, class CouplingMap, class TempMap>
void coupling(
   const Graph & g,
   double len_ref,
   WeightMap weight_map,
   CouplingMap coupling_map,
   TempMap cs_xa, TempMap cs_by)
{
   typedef typename boost::graph_traits<Graph>::edge_iterator EdgeIter;
   
   coupling_init(g, coupling_map);
   
   std::pair<EdgeIter,EdgeIter> ep=boost::edges(g);
   for (EdgeIter ei=ep.first; ei!=ep.second; ei++)
   {
      coupling_update_edge(
         g,
         *ei,
         boost::get(weight_map,*ei) / len_ref,
         true, // is_add
         coupling_map, cs_xa, cs_by);
   }
}

template <class Graph, class CouplingMap>
double coupling_without_edge(
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

} // namespace pr_bgl
