/* File: test_sebc.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>
#include <list>
#include <map>
#include <vector>
#include <stdexcept>

#include <boost/function.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/reverse_graph.hpp>

#include <pr_bgl/compose_property_map.hpp>
#include <pr_bgl/rev_edge_map.h>
#include <pr_bgl/soft_edge_bc.h>

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

double f_const(double len)
{
   return 1.0;
}

int main()
{
   printf("starting test_sebc!\n");
   
   // make a sweet 3x3 undirected 4-connected graph
   // 1 2 3
   // 4 5 6
   // 7 8 9
   // start is 7
   // goal is 3
   // all edges are weight 1
   // directed edges are RIGHT and UP
   
   Graph g;
   
   // vertices
   std::map<Vertex,int> v_to_num;
   std::map<int,Vertex> num_to_v;
   for (int vi=1; vi<=9; vi++)
   {
      Vertex v = boost::add_vertex(g);
      v_to_num[v] = vi;
      num_to_v[vi] = v;
   }
   
   // horiz
   boost::add_edge(num_to_v[1], num_to_v[2], g);
   boost::add_edge(num_to_v[2], num_to_v[3], g);
   boost::add_edge(num_to_v[4], num_to_v[5], g);
   boost::add_edge(num_to_v[5], num_to_v[6], g);
   boost::add_edge(num_to_v[7], num_to_v[8], g);
   boost::add_edge(num_to_v[8], num_to_v[9], g);
   // vert
   boost::add_edge(num_to_v[7], num_to_v[4], g);
   boost::add_edge(num_to_v[4], num_to_v[1], g);
   boost::add_edge(num_to_v[8], num_to_v[5], g);
   boost::add_edge(num_to_v[5], num_to_v[2], g);
   boost::add_edge(num_to_v[9], num_to_v[6], g);
   boost::add_edge(num_to_v[6], num_to_v[3], g);
   
   // edge weights
   std::map<Edge,double> weights;
   for (std::pair<EdgeIter,EdgeIter> ep = boost::edges(g); ep.first!=ep.second; ep.first++)
   {
      Edge e = *ep.first;
      weights[e] = 1.0;
   }
   
   // run dijkstra's!
   
   std::map<Vertex,double> goaldist;
   std::map<Vertex,Vertex> preds;
   std::map<Edge,double> scores;
   
   // run reversed dijkstra's to get goaldist (and preds)
   boost::reverse_graph<Graph> rg(g);
   boost::dijkstra_shortest_paths(
      rg,
      num_to_v[3], // source (actually dest of non-reversed graph)
      boost::make_assoc_property_map(preds),
      boost::make_assoc_property_map(goaldist),
      pr_bgl::make_compose_property_map(
         boost::make_assoc_property_map(weights),
         pr_bgl::RevEdgeMap<Graph>(rg)),
      boost::get(boost::vertex_index, g),
      std::less<double>(), // compare
      boost::closed_plus<double>(std::numeric_limits<double>::max()), // combine
      std::numeric_limits<double>::max(),
      double(),
      boost::make_dijkstra_visitor(boost::null_visitor())
      );
   
   printf("solved distances:\n");
   printf("[%.1f] [%.1f] [%.1f]\n", goaldist[num_to_v[1]], goaldist[num_to_v[2]], goaldist[num_to_v[3]]);
   printf("[%.1f] [%.1f] [%.1f]\n", goaldist[num_to_v[4]], goaldist[num_to_v[5]], goaldist[num_to_v[6]]);
   printf("[%.1f] [%.1f] [%.1f]\n", goaldist[num_to_v[7]], goaldist[num_to_v[8]], goaldist[num_to_v[9]]);
   
   printf("computing soft edge betweenness centrality ...\n");
   pr_bgl::soft_edge_bc(g, num_to_v[7], num_to_v[3],
      f_const, 4.1,
      boost::make_assoc_property_map(weights),
      boost::make_assoc_property_map(goaldist),
      boost::make_assoc_property_map(scores));
   
   // print result
   for (std::pair<EdgeIter,EdgeIter> ep=boost::edges(g); ep.first!=ep.second; ep.first++)
   {
      Edge e = *ep.first;
      printf("edge from %d to %d has score %f!\n",
         v_to_num[boost::source(e,g)],
         v_to_num[boost::target(e,g)],
         scores[e]);
   }
   
   return 0;
}
