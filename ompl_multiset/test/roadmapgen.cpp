/* File: roadmapgen.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>
#include <fstream>

#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <pr_bgl/graph_io.h>
#include <pr_bgl/string_map.h>

#include <ompl_multiset/util.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/RoadmapGen.h>
#include <ompl_multiset/RoadmapGenRGG.h>

#include <gtest/gtest.h>


class E8Roadmap
{
public:

   class StateContainer
   {
   private:
      const ompl::base::StateSpacePtr space;
   public:
      ompl::base::State * state;
      StateContainer(ompl::base::StateSpacePtr space):
         space(space), state(space->allocState()) {}
      ~StateContainer() { space->freeState(this->state); }
   };

   struct VertexProperties
   {
      boost::shared_ptr<StateContainer> state;
      int subgraph;
      bool is_shadow;
   };
   struct EdgeProperties
   {
      std::size_t index;
      double distance;
      int subgraph;
   };

   typedef boost::adjacency_list<
      boost::vecS, // Edgelist ds, for per-vertex out-edges
      boost::vecS, // VertexList ds, for vertex set
      boost::undirectedS, // type of graph
      VertexProperties, // internal (bundled) vertex properties
      EdgeProperties // internal (bundled) edge properties
      > Graph;
   
   typedef boost::graph_traits<Graph> GraphTypes;
   typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
   typedef boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
   typedef boost::graph_traits<Graph>::in_edge_iterator EdgeInIter;
   
   typedef boost::property_map< Graph, boost::vertex_index_t>::type VertexIndexMap;
   typedef boost::property_map<Graph, std::size_t EdgeProperties::*>::type EdgeIndexMap;
   
   typedef boost::vector_property_map<E8Roadmap::Edge> EdgeVectorMap;
   
   typedef boost::property_map<Graph, boost::shared_ptr<StateContainer> VertexProperties::*>::type StateMap;
   typedef boost::property_map<Graph, int VertexProperties::*>::type VertexSubgraphMap;
   typedef boost::property_map<Graph, int EdgeProperties::*>::type EdgeSubgraphMap;
   typedef boost::property_map<Graph, bool VertexProperties::*>::type IsShadowMap;
   typedef boost::property_map<Graph, double EdgeProperties::*>::type DistanceMap;
   
   typedef boost::shared_ptr< ompl_multiset::RoadmapGen<E8Roadmap> > RoadmapGenPtr;
};


// Tests factorial of 0.
TEST(RoadmapGenRRGTestCase, FixedExampleTest)
{
   ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
   space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
   E8Roadmap::RoadmapGenPtr p_mygen(new ompl_multiset::RoadmapGenRGG<E8Roadmap>(space, "n=10 radius=0.3 seed=1"));
   
   E8Roadmap::Graph g;
   E8Roadmap::EdgeVectorMap edge_vector(boost::num_edges(g));
   
   // generate a graph
   p_mygen->generate(g,
      get(boost::vertex_index, g),
      get(&E8Roadmap::EdgeProperties::index, g),
      edge_vector,
      1,
      get(&E8Roadmap::VertexProperties::state, g),
      get(&E8Roadmap::EdgeProperties::distance, g),
      get(&E8Roadmap::VertexProperties::subgraph, g),
      get(&E8Roadmap::EdgeProperties::subgraph, g),
      get(&E8Roadmap::VertexProperties::is_shadow, g));
   
   // write it out to file
   pr_bgl::GraphIO<E8Roadmap::Graph, E8Roadmap::VertexIndexMap, E8Roadmap::EdgeIndexMap, E8Roadmap::EdgeVectorMap>
      io(g,
         get(boost::vertex_index, g),
         get(&E8Roadmap::EdgeProperties::index, g),
         edge_vector);

   io.add_property_map("distance", pr_bgl::make_string_map(get(&E8Roadmap::EdgeProperties::distance,g)));
   
   std::stringstream ss;
   
   io.dump_graph(ss);
   io.dump_properties(ss);
   
   EXPECT_EQ(ss.str(), std::string()
      + "vertex 0\n"
      + "vertex 1\n"
      + "vertex 2\n"
      + "vertex 3\n"
      + "vertex 4\n"
      + "vertex 5\n"
      + "vertex 6\n"
      + "vertex 7\n"
      + "vertex 8\n"
      + "vertex 9\n"
      + "edge 0 source 3 target 0\n"
      + "edge 1 source 4 target 2\n"
      + "edge 2 source 5 target 2\n"
      + "edge 3 source 5 target 4\n"
      + "edge 4 source 6 target 4\n"
      + "edge 5 source 6 target 5\n"
      + "edge 6 source 8 target 0\n"
      + "edge 7 source 8 target 3\n"
      + "edge 8 source 8 target 7\n"
      + "edge 9 source 9 target 0\n"
      + "edge 10 source 9 target 1\n"
      + "edge 11 source 9 target 3\n"
      + "edge 12 source 9 target 7\n"
      + "edge 13 source 9 target 8\n"
      + "edge 0 distance 0.1147044426379449\n"
      + "edge 1 distance 0.18209907275620318\n"
      + "edge 2 distance 0.2838557364722314\n"
      + "edge 3 distance 0.16946635062048906\n"
      + "edge 4 distance 0.15687714686420182\n"
      + "edge 5 distance 0.09432093383515884\n"
      + "edge 6 distance 0.06488792402680842\n"
      + "edge 7 distance 0.11379975283336656\n"
      + "edge 8 distance 0.2706807465967623\n"
      + "edge 9 distance 0.1938991690396887\n"
      + "edge 10 distance 0.20095650007646965\n"
      + "edge 11 distance 0.2815157007025205\n"
      + "edge 12 distance 0.2617690590175142\n"
      + "edge 13 distance 0.16774879052012587\n"
   );
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
