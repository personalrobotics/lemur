/* File: roadmapgen.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>
#include <fstream>

#include <boost/property_map/property_map.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <pr_bgl/graph_io.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/edge_indexed_graph.h>

#include <ompl_multiset/util.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapRGG.h>

#include <gtest/gtest.h>


struct StateContainer
{
   const ompl::base::StateSpace * space;
   ompl::base::State * state;
   StateContainer(ompl::base::StateSpace * space):
      space(space), state(space->allocState()) {}
   ~StateContainer() { space->freeState(this->state); }
};
typedef boost::shared_ptr<StateContainer> StateContainerPtr;

struct VertexProperties
{
   StateContainerPtr state;
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

typedef boost::graph_traits<Graph>::edge_descriptor Edge;

typedef boost::property_map<Graph, boost::vertex_index_t>::type VertexIndexMap;
typedef boost::property_map<Graph, std::size_t EdgeProperties::*>::type EdgeIndexMap;
typedef boost::vector_property_map<Edge> EdgeVectorMap;

typedef boost::property_map<Graph, StateContainerPtr VertexProperties::*>::type StateMap;
typedef boost::property_map<Graph, int VertexProperties::*>::type VertexSubgraphMap;
typedef boost::property_map<Graph, int EdgeProperties::*>::type EdgeSubgraphMap;
typedef boost::property_map<Graph, bool VertexProperties::*>::type IsShadowMap;
typedef boost::property_map<Graph, double EdgeProperties::*>::type DistanceMap;

typedef pr_bgl::EdgeIndexedGraph<Graph, EdgeIndexMap> EdgeIndexedGraph;
typedef ompl_multiset::NNLinear<EdgeIndexedGraph,StateMap> NN;
typedef ompl_multiset::Roadmap<EdgeIndexedGraph,StateMap,DistanceMap,VertexSubgraphMap,EdgeSubgraphMap,IsShadowMap,NN> Roadmap;
typedef boost::shared_ptr<Roadmap> RoadmapPtr;


TEST(RoadmapRRGTestCase, FixedExampleTest)
{
   ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
   space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
   RoadmapPtr p_mygen(new ompl_multiset::RoadmapRGG<Roadmap>(space, 10, 0.3, 1));
   
   Graph g;
   
   pr_bgl::EdgeIndexedGraph<Graph, EdgeIndexMap>
      eig(g, get(&EdgeProperties::index, g));
   
   // generate a graph
   NN nnlin(eig, get(&VertexProperties::state,g), space);
   p_mygen->generate(eig, nnlin,
      get(&VertexProperties::state, g),
      get(&EdgeProperties::distance, g),
      get(&VertexProperties::subgraph, g),
      get(&EdgeProperties::subgraph, g),
      get(&VertexProperties::is_shadow, g));
   
   // write it out to file
   boost::dynamic_properties props;
   props.property("distance", pr_bgl::make_string_map(get(&EdgeProperties::distance,g)));
   std::stringstream ss;
   pr_bgl::write_graphio_graph(ss, g,
      get(boost::vertex_index, g), get(&EdgeProperties::index, g));
   pr_bgl::write_graphio_properties(ss, g,
      get(boost::vertex_index, g), get(&EdgeProperties::index, g),
      props);
   
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
      + "eprop 0 distance 0.1147044426379449\n"
      + "eprop 1 distance 0.18209907275620318\n"
      + "eprop 2 distance 0.2838557364722314\n"
      + "eprop 3 distance 0.16946635062048906\n"
      + "eprop 4 distance 0.15687714686420182\n"
      + "eprop 5 distance 0.09432093383515884\n"
      + "eprop 6 distance 0.06488792402680842\n"
      + "eprop 7 distance 0.11379975283336656\n"
      + "eprop 8 distance 0.2706807465967623\n"
      + "eprop 9 distance 0.1938991690396887\n"
      + "eprop 10 distance 0.20095650007646965\n"
      + "eprop 11 distance 0.2815157007025205\n"
      + "eprop 12 distance 0.2617690590175142\n"
      + "eprop 13 distance 0.16774879052012587\n"
   );
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
