/* File: generate_unit_roadmap.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <algorithm>
#include <fstream>

#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <pr_bgl/graph_io.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/edge_indexed_graph.h>

#include <ompl_multiset/util.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/RoadmapGen.h>
#include <ompl_multiset/RoadmapGenRGG.h>
#include <ompl_multiset/RoadmapGenHalton.h>


class GraphTypes
{
public:

   struct StateContainer
   {
      const ompl::base::StateSpacePtr space;
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
   
   typedef boost::graph_traits<Graph>::edge_descriptor Edge;
   
   typedef boost::property_map<Graph, boost::vertex_index_t>::type VertexIndexMap;
   typedef boost::property_map<Graph, std::size_t EdgeProperties::*>::type EdgeIndexMap;
   typedef boost::vector_property_map<Edge> EdgeVectorMap;
   
   typedef boost::property_map<Graph, boost::shared_ptr<StateContainer> VertexProperties::*>::type StateMap;
   typedef boost::property_map<Graph, int VertexProperties::*>::type VertexSubgraphMap;
   typedef boost::property_map<Graph, int EdgeProperties::*>::type EdgeSubgraphMap;
   typedef boost::property_map<Graph, bool VertexProperties::*>::type IsShadowMap;
   typedef boost::property_map<Graph, double EdgeProperties::*>::type DistanceMap;
};

class EdgeIndexedGraphTypes
{
public:
   typedef pr_bgl::EdgeIndexedGraph<GraphTypes::Graph, GraphTypes::EdgeIndexMap> Graph;
   typedef GraphTypes::StateMap StateMap;
   typedef GraphTypes::DistanceMap DistanceMap;
   typedef GraphTypes::VertexSubgraphMap VertexSubgraphMap;
   typedef GraphTypes::EdgeSubgraphMap EdgeSubgraphMap;
   typedef GraphTypes::IsShadowMap IsShadowMap;
   typedef boost::shared_ptr< ompl_multiset::RoadmapGen<EdgeIndexedGraphTypes> > RoadmapGenPtr;
   typedef GraphTypes::StateContainer StateContainer;
};


inline void stringify_from_x(std::string & repr, const boost::shared_ptr<GraphTypes::StateContainer> & in)
{
   unsigned int dim = in->space->getDimension();
   ompl::base::RealVectorStateSpace::StateType * state
      = in->state->as<ompl::base::RealVectorStateSpace::StateType>();
   repr.clear();
   for (unsigned int ui=0; ui<dim; ui++)
   {
      if (ui)
         repr += " ";
      std::string component_repr;
      pr_bgl::stringify_from_x(component_repr, state->values[ui]);
      repr += component_repr;
   }
}

inline void stringify_to_x(const std::string & in, boost::shared_ptr<GraphTypes::StateContainer> & repr)
{
   repr.reset();
   //repr = atof(in.c_str());
}


int main(int argc, char **argv)
{
   if (argc != 4)
   {
      printf("Usage: generate_unit_roadmap <dim> <roadmap-type> '<roadmap-args>'\n");
      return 1;
   }
   
   int dim = atoi(argv[1]);
   printf("creating unit ompl space of dimension %d ...\n", dim);
   ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dim));
   space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
   
   EdgeIndexedGraphTypes::RoadmapGenPtr p_mygen;
   
   std::string roadmap_type(argv[2]);
   std::transform(roadmap_type.begin(), roadmap_type.end(), roadmap_type.begin(), ::tolower);
   printf("creating roadmap of type %s ...\n", roadmap_type.c_str());
   if (roadmap_type == "rgg")
      p_mygen.reset(new ompl_multiset::RoadmapGenRGG<EdgeIndexedGraphTypes>(space, std::string(argv[3])));
   else if (roadmap_type == "halton")
      p_mygen.reset(new ompl_multiset::RoadmapGenHalton<EdgeIndexedGraphTypes>(space, std::string(argv[3])));
   else
   {
      printf("unknown roadmap type!\n");
      return 1;
   }
   
   GraphTypes::Graph g;
   
   pr_bgl::EdgeIndexedGraph<GraphTypes::Graph, GraphTypes::EdgeIndexMap>
      eig(g,
         get(&GraphTypes::EdgeProperties::index, g)
         );
   
   
   // generate a graph
   p_mygen->generate(eig,
      1,
      get(&GraphTypes::VertexProperties::state, g),
      get(&GraphTypes::EdgeProperties::distance, g),
      get(&GraphTypes::VertexProperties::subgraph, g),
      get(&GraphTypes::EdgeProperties::subgraph, g),
      get(&GraphTypes::VertexProperties::is_shadow, g));
   
   // write it out to file
   pr_bgl::GraphIO<GraphTypes::Graph, GraphTypes::VertexIndexMap, GraphTypes::EdgeIndexMap, GraphTypes::EdgeVectorMap>
      io(g,
         get(boost::vertex_index, g),
         get(&GraphTypes::EdgeProperties::index, g),
         eig.edge_vector_map);

   io.add_property_map("state", pr_bgl::make_string_map(get(&GraphTypes::VertexProperties::state,g)));
   io.add_property_map("subgraph", pr_bgl::make_string_map(get(&GraphTypes::VertexProperties::subgraph,g)));
   io.add_property_map("subgraph", pr_bgl::make_string_map(get(&GraphTypes::EdgeProperties::subgraph,g)));
   io.add_property_map("is_shadow", pr_bgl::make_string_map(get(&GraphTypes::VertexProperties::is_shadow,g)));
   io.add_property_map("distance", pr_bgl::make_string_map(get(&GraphTypes::EdgeProperties::distance,g)));
   
   io.dump_graph(std::cout);
   io.dump_properties(std::cout);
   
   return 0;
}
