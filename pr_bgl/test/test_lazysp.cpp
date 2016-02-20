/* File: lazysp.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>
#include <map>
#include <string>
#include <sstream>

#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <pr_bgl/flag_set_map.h>
#include <pr_bgl/lazysp.h>
#include <pr_bgl/lazysp_incsp_dijkstra.h>

#include <gtest/gtest.h>

#define XSTR(s) STR(s)
#define STR(s) # s

TEST(LazySPTestCase, LazySPTest)
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
   
   Graph g;
   std::map<Vertex, std::string> state;
   std::map<Edge, double> dist;
   std::map<Edge, bool> isevaled;
   
   // read the test graph
   std::ifstream fp;
   fp.open(XSTR(DATADIR) "/halton2d.xml");
   boost::dynamic_properties props;
   props.property("state", boost::make_assoc_property_map(state));
   boost::read_graphml(fp, g, props);
   ASSERT_EQ(30, num_vertices(g));
   ASSERT_EQ(98, num_edges(g));
   
   // compute distances
   EdgeIter ei, ei_end;
   for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ++ei)
   {
      Vertex v1 = source(*ei, g);
      Vertex v2 = target(*ei, g);
      double v1x, v1y, v2x, v2y;
      std::stringstream ss(state[v1] + " " + state[v2]);
      ss >> v1x >> v1y >> v2x >> v2y;
      dist[*ei] = sqrt(pow(v2x-v1x,2.) + pow(v2y-v1y,2.));
   }
   
   // run lazysp
   std::vector<Edge> path;
   std::map<Edge, double> dist_lazy = dist;
   std::vector<Vertex> v_startpreds(num_vertices(g));
   std::vector<double> v_startdist(num_vertices(g));
   
   bool success = pr_bgl::lazysp(
      g, vertex(17,g), vertex(22,g),
      pr_bgl::make_flag_set_map(
         boost::make_assoc_property_map(dist), 
         boost::make_assoc_property_map(isevaled)),
      boost::make_assoc_property_map(dist_lazy),
      boost::make_assoc_property_map(isevaled),
      path,
      pr_bgl::make_lazysp_incsp_dijkstra< Graph, boost::associative_property_map< std::map<Edge,double> > >(
         boost::make_iterator_property_map(v_startpreds.begin(), get(boost::vertex_index,g)), // startpreds_map
         boost::make_iterator_property_map(v_startdist.begin(), get(boost::vertex_index,g))), // startdist_map
      pr_bgl::lazysp_selector_alt(),
      pr_bgl::lazysp_visitor_null());
   ASSERT_EQ(true, success);
   
   // validate path
   ASSERT_EQ(5, path.size());
   ASSERT_EQ(17, source(path[0],g));
   ASSERT_EQ( 5, target(path[0],g));
   ASSERT_EQ( 0, target(path[1],g));
   ASSERT_EQ(12, target(path[2],g));
   ASSERT_EQ(10, target(path[3],g));
   ASSERT_EQ(22, target(path[4],g));
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
