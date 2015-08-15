/* File: test_coupling.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>

#include <boost/graph/filtered_graph.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/type_traits/is_base_and_derived.hpp>
#include <boost/type_traits/is_same.hpp>

#include <pr_bgl/coupling.h>

// single vertex
int test1()
{
   typedef boost::adjacency_list<
      boost::vecS, // Edgelist ds, for per-vertex out-edges
      boost::vecS, // VertexList ds, for vertex set
      boost::undirectedS // type of graph
      > Graph;
   typedef boost::graph_traits<Graph> GraphTypes;
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
   typedef boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   typedef boost::graph_traits<Graph>::in_edge_iterator EdgeInIter;
   
   printf("starting test 1 (single vertex):\n");
   
   Graph g;
   
   Vertex v = boost::add_vertex(g);
   
   std::map<Edge,double> weights;
   
   std::map<std::pair<Vertex,Vertex>, double> coupling_map;
   
   std::map<Vertex, double> cs_xs;
   std::map<Vertex, double> cs_ty;
   
   pr_bgl::coupling(g, 1.0,
      boost::make_assoc_property_map(weights),
      boost::make_assoc_property_map(coupling_map),
      boost::make_assoc_property_map(cs_xs),
      boost::make_assoc_property_map(cs_ty));
   
   for (std::pair<VertexIter,VertexIter> vpx=boost::vertices(g);
      vpx.first!=vpx.second; vpx.first++)
   {
      Vertex v_x = *vpx.first;
      for (std::pair<VertexIter,VertexIter> vpy=boost::vertices(g);
         vpy.first!=vpy.second; vpy.first++)
      {
         Vertex v_y = *vpy.first;
         printf("coupling: %f\n", 
            coupling_map[std::make_pair(v_x,v_y)]);
      }
   }
   
   return 0;
}

// single directed edge
int test2()
{
   typedef boost::adjacency_list<
      boost::vecS, // Edgelist ds, for per-vertex out-edges
      boost::vecS, // VertexList ds, for vertex set
      boost::directedS // type of graph
      > Graph;
   typedef boost::graph_traits<Graph> GraphTypes;
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
   typedef boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   typedef boost::graph_traits<Graph>::in_edge_iterator EdgeInIter;
   
   printf("starting test 2 (single directed edge):\n");
   
   Graph g;
   
   // add vertices
   Vertex v1 = boost::add_vertex(g);
   Vertex v2 = boost::add_vertex(g);
   std::map<Vertex, std::string> v_names;
   v_names[v1] = "v1";
   v_names[v2] = "v2";
   
   // add edges
   Edge e12 = boost::add_edge(v1, v2, g).first;
   std::map<Edge,double> weights;
   weights[e12] = 1.0;
   
   // output, temp
   std::map<std::pair<Vertex,Vertex>, double> coupling_map;
   std::map<Vertex, double> cs_xs;
   std::map<Vertex, double> cs_ty;
   
   pr_bgl::coupling(g, 1.0,
      boost::make_assoc_property_map(weights),
      boost::make_assoc_property_map(coupling_map),
      boost::make_assoc_property_map(cs_xs),
      boost::make_assoc_property_map(cs_ty));
   
   for (std::pair<VertexIter,VertexIter> vpx=boost::vertices(g);
      vpx.first!=vpx.second; vpx.first++)
   {
      Vertex v_x = *vpx.first;
      for (std::pair<VertexIter,VertexIter> vpy=boost::vertices(g);
         vpy.first!=vpy.second; vpy.first++)
      {
         Vertex v_y = *vpy.first;
         printf("coupling %s->%s: %f\n",
            v_names[v_x].c_str(), v_names[v_y].c_str(),
            coupling_map[std::make_pair(v_x,v_y)]);
      }
   }
   
   return 0;
}

// double directed edges
int test3()
{
   typedef boost::adjacency_list<
      boost::vecS, // Edgelist ds, for per-vertex out-edges
      boost::vecS, // VertexList ds, for vertex set
      boost::directedS // type of graph
      > Graph;
   typedef boost::graph_traits<Graph> GraphTypes;
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
   typedef boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   typedef boost::graph_traits<Graph>::in_edge_iterator EdgeInIter;
   
   printf("starting test 3 (double directed edges):\n");
   
   Graph g;
   
   // add vertices
   Vertex v1 = boost::add_vertex(g);
   Vertex v2 = boost::add_vertex(g);
   std::map<Vertex, std::string> v_names;
   v_names[v1] = "v1";
   v_names[v2] = "v2";
   
   // add edges
   Edge e12 = boost::add_edge(v1, v2, g).first;
   Edge e21 = boost::add_edge(v2, v1, g).first;
   std::map<Edge,double> weights;
   weights[e12] = 1.0;
   weights[e21] = 1.0;
   
   // output, temp
   std::map<std::pair<Vertex,Vertex>, double> coupling_map;
   std::map<Vertex, double> cs_xs;
   std::map<Vertex, double> cs_ty;
   
   pr_bgl::coupling(g, 1.0,
      boost::make_assoc_property_map(weights),
      boost::make_assoc_property_map(coupling_map),
      boost::make_assoc_property_map(cs_xs),
      boost::make_assoc_property_map(cs_ty));
   
   for (std::pair<VertexIter,VertexIter> vpx=boost::vertices(g);
      vpx.first!=vpx.second; vpx.first++)
   {
      Vertex v_x = *vpx.first;
      for (std::pair<VertexIter,VertexIter> vpy=boost::vertices(g);
         vpy.first!=vpy.second; vpy.first++)
      {
         Vertex v_y = *vpy.first;
         printf("coupling %s->%s: %f\n",
            v_names[v_x].c_str(), v_names[v_y].c_str(),
            coupling_map[std::make_pair(v_x,v_y)]);
      }
   }
   
   return 0;
}

// single undirected edge
int test4()
{
   typedef boost::adjacency_list<
      boost::vecS, // Edgelist ds, for per-vertex out-edges
      boost::vecS, // VertexList ds, for vertex set
      boost::undirectedS // type of graph
      > Graph;
   typedef boost::graph_traits<Graph> GraphTypes;
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
   typedef boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   typedef boost::graph_traits<Graph>::in_edge_iterator EdgeInIter;
   
   printf("starting test 4 (single undirected edge):\n");
   
   Graph g;
   
   // add vertices
   Vertex v1 = boost::add_vertex(g);
   Vertex v2 = boost::add_vertex(g);
   std::map<Vertex, std::string> v_names;
   v_names[v1] = "v1";
   v_names[v2] = "v2";
   
   // add edges
   Edge e12 = boost::add_edge(v1, v2, g).first;
   std::map<Edge,double> weights;
   weights[e12] = 1.0;
   
   // output, temp
   std::map<std::pair<Vertex,Vertex>, double> coupling_map;
   std::map<Vertex, double> cs_xs;
   std::map<Vertex, double> cs_ty;
   
   pr_bgl::coupling(g, 1.0,
      boost::make_assoc_property_map(weights),
      boost::make_assoc_property_map(coupling_map),
      boost::make_assoc_property_map(cs_xs),
      boost::make_assoc_property_map(cs_ty));
   
   for (std::pair<VertexIter,VertexIter> vpx=boost::vertices(g);
      vpx.first!=vpx.second; vpx.first++)
   {
      Vertex v_x = *vpx.first;
      for (std::pair<VertexIter,VertexIter> vpy=boost::vertices(g);
         vpy.first!=vpy.second; vpy.first++)
      {
         Vertex v_y = *vpy.first;
         printf("coupling %s->%s: %f\n",
            v_names[v_x].c_str(), v_names[v_y].c_str(),
            coupling_map[std::make_pair(v_x,v_y)]);
      }
   }
   
   return 0;
}

int main()
{
   printf("starting test_coupling!\n");
   
   test1();
   test2();
   test3();
   test4();
   
   return 0;
}
