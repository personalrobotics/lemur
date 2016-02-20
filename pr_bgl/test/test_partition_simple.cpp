/* File: partition_simple.cpp
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
#include <pr_bgl/partition_simple.h>

#include <gtest/gtest.h>

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

TEST(PartitionSimpleTestCase, PartitionSimpleTest)
{
   printf("starting partition simple test!\n");
   
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
   Edge e12 = add_edge(num_to_v[1], num_to_v[2], g).first;
   Edge e23 = add_edge(num_to_v[2], num_to_v[3], g).first;
   Edge e45 = add_edge(num_to_v[4], num_to_v[5], g).first;
   Edge e56 = add_edge(num_to_v[5], num_to_v[6], g).first;
   Edge e78 = add_edge(num_to_v[7], num_to_v[8], g).first;
   Edge e89 = add_edge(num_to_v[8], num_to_v[9], g).first;
   // vert
   Edge e74 = add_edge(num_to_v[7], num_to_v[4], g).first;
   Edge e41 = add_edge(num_to_v[4], num_to_v[1], g).first;
   Edge e85 = add_edge(num_to_v[8], num_to_v[5], g).first;
   Edge e52 = add_edge(num_to_v[5], num_to_v[2], g).first;
   Edge e96 = add_edge(num_to_v[9], num_to_v[6], g).first;
   Edge e63 = add_edge(num_to_v[6], num_to_v[3], g).first;
   
   // edge weights
   std::map<Edge,double> weights;
   for (std::pair<EdgeIter,EdgeIter> ep = edges(g); ep.first!=ep.second; ep.first++)
      weights[*ep.first] = 1.0;
   
   // run dijkstra's!
   
   std::map<Vertex,double> goaldist;
   std::map<Vertex,Vertex> preds;
   std::map<Vertex,bool> isused;
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
         pr_bgl::rev_edge_map<Graph>(rg)),
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

   ASSERT_EQ(2.0, goaldist[num_to_v[1]]);
      ASSERT_EQ(1.0, goaldist[num_to_v[2]]);
         ASSERT_EQ(0.0, goaldist[num_to_v[3]]);
   ASSERT_EQ(3.0, goaldist[num_to_v[4]]);
      ASSERT_EQ(2.0, goaldist[num_to_v[5]]);
         ASSERT_EQ(1.0, goaldist[num_to_v[6]]);
   ASSERT_EQ(4.0, goaldist[num_to_v[7]]);
      ASSERT_EQ(3.0, goaldist[num_to_v[8]]);
         ASSERT_EQ(2.0, goaldist[num_to_v[9]]);

   printf("computing simple partition function ...\n");
   double score_total = pr_bgl::partition_simple(
      g, num_to_v[7], num_to_v[3],
      1.0, 4.1,
      boost::make_assoc_property_map(weights),
      boost::make_assoc_property_map(goaldist),
      boost::make_assoc_property_map(scores),
      boost::make_assoc_property_map(isused));
   
   for (std::pair<EdgeIter,EdgeIter> ep = edges(g); ep.first!=ep.second; ep.first++)
      scores[*ep.first] /= score_total;
   
   printf("\n");
   printf("  [1]--%.3f->[2]--%.3f->[3]\n", scores[e12], scores[e23]);
   printf("   ^           ^           ^\n");
   printf(" %.3f       %.3f       %.3f\n", scores[e41], scores[e52], scores[e63]);
   printf("   |           |           |\n");
   printf("  [4]--%.3f->[5]--%.3f->[6]\n", scores[e45], scores[e56]);
   printf("   ^           ^           ^\n");
   printf(" %.3f       %.3f       %.3f\n", scores[e74], scores[e85], scores[e96]);
   printf("   |           |           |\n");
   printf("  [7]--%.3f->[8]--%.3f->[9]\n", scores[e78], scores[e89]);
   printf("\n");
   
   ASSERT_EQ(1.0/6.0, scores[e12]);
   ASSERT_EQ(3.0/6.0, scores[e23]);
   
   ASSERT_EQ(1.0/6.0, scores[e41]);
   ASSERT_EQ(2.0/6.0, scores[e52]);
   ASSERT_EQ(3.0/6.0, scores[e63]);
   
   ASSERT_EQ(2.0/6.0, scores[e45]);
   ASSERT_EQ(2.0/6.0, scores[e56]);
   
   ASSERT_EQ(3.0/6.0, scores[e74]);
   ASSERT_EQ(2.0/6.0, scores[e85]);
   ASSERT_EQ(1.0/6.0, scores[e96]);
   
   ASSERT_EQ(3.0/6.0, scores[e78]);
   ASSERT_EQ(1.0/6.0, scores[e89]);
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
