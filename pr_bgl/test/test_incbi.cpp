/* File: incbi.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <stdio.h>

#include <boost/graph/relax.hpp> // for closed_plus
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <pr_bgl/heap_indexed.h>
#include <pr_bgl/incbi.h>

#include <gtest/gtest.h>

TEST(IncBiTestCase, IncBiTest)
{
   /*        [20]    v2    [20]
    *             /      \ 
    * v0 ------ v1 ------ v3 ------ v4
    *     [20]      [30]      [20]
    */
   
   // create a typedef for the Graph type
   typedef boost::adjacency_list<
      boost::vecS, boost::vecS, boost::undirectedS> Graph;
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   
   typedef boost::iterator_property_map<
      std::vector<Vertex>::iterator,
      boost::property_map<Graph, boost::vertex_index_t>::type > VertexVertexMap;
   typedef boost::iterator_property_map<
      std::vector<double>::iterator,
      boost::property_map<Graph, boost::vertex_index_t>::type > VertexWeightMap;
      typedef boost::associative_property_map< std::map<Edge,size_t> > EdgeIndexMap;
   typedef boost::associative_property_map< std::map<Edge,double> > EdgeWeightMap;

   Graph g(5);
   std::map<Edge, double> edge_weights;
   std::map<Edge, size_t> edge_indices;
   Edge e01 = add_edge(0, 1, g).first;
   Edge e12 = add_edge(1, 2, g).first;
   Edge e13 = add_edge(1, 3, g).first;
   Edge e23 = add_edge(2, 3, g).first;
   Edge e34 = add_edge(3, 4, g).first;
   edge_indices[e01] = 0;
   edge_indices[e12] = 1;
   edge_indices[e13] = 2;
   edge_indices[e23] = 3;
   edge_indices[e34] = 4;
   edge_weights[e01] = 20.0;
   edge_weights[e12] = 20.0;
   edge_weights[e13] = 30.0;
   edge_weights[e23] = 20.0;
   edge_weights[e34] = 20.0;
   
   std::vector<Vertex> start_predecessor(num_vertices(g));
   std::vector<double> start_dist(num_vertices(g));
   std::vector<double> start_dist_lookahead(num_vertices(g));
   std::vector<Vertex> goal_predecessor(num_vertices(g));
   std::vector<double> goal_dist(num_vertices(g));
   std::vector<double> goal_dist_lookahead(num_vertices(g));
   
   VertexVertexMap start_predecessor_map(start_predecessor.begin(), get(boost::vertex_index,g));
   VertexWeightMap start_dist_map(start_dist.begin(), get(boost::vertex_index,g));
   VertexWeightMap start_dist_lookahead_map(start_dist_lookahead.begin(), get(boost::vertex_index,g));
   VertexVertexMap goal_predecessor_map(goal_predecessor.begin(), get(boost::vertex_index,g));
   VertexWeightMap goal_dist_map(goal_dist.begin(), get(boost::vertex_index,g));
   VertexWeightMap goal_dist_lookahead_map(goal_dist_lookahead.begin(), get(boost::vertex_index,g));
   
   // create incbi instance
   pr_bgl::incbi<Graph,
      VertexVertexMap,VertexWeightMap,VertexWeightMap,
      VertexVertexMap,VertexWeightMap,VertexWeightMap,
      EdgeWeightMap,
      boost::property_map<Graph, boost::vertex_index_t>::type, EdgeIndexMap,
      std::less<double>, // compare
      boost::closed_plus<double>, // combine
      double,
      double,
      pr_bgl::incbi_visitor_null<Graph>,
      pr_bgl::incbi_balancer_distance<Vertex,double>
      > myincbi(
         g, 0, 4,
         start_predecessor_map, start_dist_map, start_dist_lookahead_map,
         goal_predecessor_map, goal_dist_map, goal_dist_lookahead_map,
         boost::make_assoc_property_map(edge_weights),
         get(boost::vertex_index,g), boost::make_assoc_property_map(edge_indices),
         std::less<double>(), // compare
         boost::closed_plus<double>(std::numeric_limits<double>::max()), // combine
         std::numeric_limits<double>::max(),
         double(),
         0.0,
         pr_bgl::incbi_visitor_null<Graph>(),
         pr_bgl::incbi_balancer_distance<Vertex,double>());
   
   printf("computing shortest path ...\n");
   std::pair<size_t,bool> ret = myincbi.compute_shortest_path();
   ASSERT_EQ(true, ret.second);
   
   printf("eidx_connection: %lu\n", ret.first);
   ASSERT_EQ(ret.first, edge_indices[e13]);
   
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
