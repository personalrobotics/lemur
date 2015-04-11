/* File: roadmaps-2d.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapSampledConst.h>
#include <ompl_multiset/RoadmapSampledDensified.h>

#include <ompl_multiset/Cache.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>

// borrowed from http://programmingexamples.net/wiki/Boost/BGL/DijkstraUndirected

int main(int argc, char * argv[])
{
   if (argc != 4)
   {
      printf("Usage: %s <gamma_rel> <batch_size> <n_batches>\n", argv[0]);
      return 1;
   }
   double gamma_rel = atof(argv[1]);
   int batch_size = atoi(argv[2]);
   int n_batches = atoi(argv[3]);
   
   printf("starting roadmaps-2d ...\n");
   
   boost::shared_ptr<ompl::base::RealVectorStateSpace> space(new 
      ompl::base::RealVectorStateSpace(2));
   
   /* state space set bounds */
   {
      ompl::base::RealVectorBounds bounds(2);
      bounds.setLow(0, 0.0); bounds.setHigh(0, 1.0);
      bounds.setLow(1, 0.0); bounds.setHigh(1, 1.0);
      space->setBounds(bounds);
   }
   
   /* set space resolution */
   //space.setLongestValidSegmentFraction(0.01 / space->getMaximumExtent());
   
   ompl_multiset::RoadmapSampledDensified roadmap(space, 1, batch_size, gamma_rel);
   
   
#if 1
   roadmap.subgraphs_generate(n_batches);
   
   ompl_multiset::Cache * cache = ompl_multiset::cache_create(".");
   cache->roadmap_save(&roadmap);
   
#else



   
   // make a graph and do some sweet length finding
   
   
   typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
   
   typedef boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS,
      boost::no_property, EdgeWeightProperty > Graph;
   
   typedef boost::graph_traits < Graph >::vertex_descriptor vertex_descriptor;
   typedef boost::graph_traits < Graph >::edge_descriptor edge_descriptor;
   typedef std::pair<int, int> Edge;
   
   Graph g;
   
   std::vector<vertex_descriptor> g_vertices;
   std::vector<edge_descriptor> g_edges;
   
   double dist_actual = 1.0;
   for (unsigned int si=0; si<n_batches; si++)
   {
      //printf("si: %u\n", si);
      
      roadmap.subgraphs_generate(si+1);
      
      // add new vertices
      while (g_vertices.size() < roadmap.vertices.size())
         g_vertices.push_back(boost::add_vertex(g));
      
      if (si==0)
         dist_actual = space->distance(roadmap.vertices[0], roadmap.vertices[1]);
      
      // add new edges with weights
      while (g_edges.size() < roadmap.edges.size())
      {
         unsigned int vi1 = roadmap.edges[g_edges.size()].first;
         unsigned int vi2 = roadmap.edges[g_edges.size()].second;
         double dist = space->distance(roadmap.vertices[vi1], roadmap.vertices[vi2]);
         vertex_descriptor v1 = g_vertices[vi1];
         vertex_descriptor v2 = g_vertices[vi2];
         EdgeWeightProperty weight = dist;
         std::pair<edge_descriptor,bool> ret = boost::add_edge(v1, v2, weight, g);
         g_edges.push_back(ret.first);
      }
      
      {
         // Create things for Dijkstra
         std::vector<vertex_descriptor> parents(boost::num_vertices(g)); // To store parents
         std::vector<double> distances(boost::num_vertices(g), INFINITY); // To store distances
         boost::dijkstra_shortest_paths(g, g_vertices[0],
            boost::predecessor_map(&parents[0]).distance_map(&distances[0]));
         if (distances[1] == std::numeric_limits<double>::max())
            printf("inf!\n");
         else
            printf(" graph distance from v0 to v1: %f (%f%%)\n",
               distances[1], 100*distances[1]/dist_actual);
      }
   }
   
   
#endif


   
   
   
   
   
   return 0;
}
