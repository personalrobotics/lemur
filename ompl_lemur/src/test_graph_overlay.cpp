/* File: test_graph_overlay.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <algorithm>
#include <fstream>

#include <boost/property_map/property_map.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include <pr_bgl/graph_io.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/overlay_manager.h>
#include <pr_bgl/edge_indexed_graph.h>

#include <ompl_lemur/util.h>
#include <ompl_lemur/rvstate_map_string_adaptor.h>
#include <ompl_lemur/SamplerGenMonkeyPatch.h>
#include <ompl_lemur/NearestNeighborsLinearBGL.h>
#include <ompl_lemur/Roadmap.h>
#include <ompl_lemur/RoadmapHalton.h>

struct VertexProperties
{
   ompl::base::State * state;
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

typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

typedef boost::property_map<Graph, boost::vertex_index_t>::type VertexIndexMap;
typedef boost::property_map<Graph, std::size_t EdgeProperties::*>::type EdgeIndexMap;
typedef boost::vector_property_map<Edge> EdgeVectorMap;

typedef boost::property_map<Graph, ompl::base::State * VertexProperties::*>::type StateMap;
typedef boost::property_map<Graph, int VertexProperties::*>::type VertexSubgraphMap;
typedef boost::property_map<Graph, int EdgeProperties::*>::type EdgeSubgraphMap;
typedef boost::property_map<Graph, bool VertexProperties::*>::type IsShadowMap;
typedef boost::property_map<Graph, double EdgeProperties::*>::type DistanceMap;

// create an overlay graph for roots
struct OverVertexProperties
{
   VertexProperties core_properties;
   Vertex core_vertex;
};
struct OverEdgeProperties
{
   EdgeProperties core_properties;
   Edge core_edge;
};

typedef boost::adjacency_list<
   boost::vecS, // Edgelist ds, for per-vertex out-edges
   boost::listS, // VertexList ds, for vertex set
   boost::undirectedS, // type of graph
   OverVertexProperties, // internal (bundled) vertex properties
   OverEdgeProperties // internal (bundled) edge properties
   > OverGraph;

typedef boost::graph_traits<OverGraph>::vertex_descriptor OverVertex;
typedef boost::graph_traits<OverGraph>::edge_descriptor OverEdge;

typedef pr_bgl::edge_indexed_graph<Graph, EdgeIndexMap> EdgeIndexedGraph;
typedef ompl_lemur::NearestNeighborsLinearBGL<EdgeIndexedGraph,StateMap> NN;
typedef ompl_lemur::RoadmapArgs<EdgeIndexedGraph,StateMap,DistanceMap,VertexSubgraphMap,EdgeSubgraphMap,IsShadowMap,EdgeVectorMap,NN> RoadmapArgs;
typedef boost::shared_ptr< ompl_lemur::RoadmapHalton<RoadmapArgs> > RoadmapPtr;

int main(int argc, char **argv)
{
   printf("starting test-graph-overlay\n");
   
   ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
   space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
   
   // graph
   Graph g;
   
   pr_bgl::edge_indexed_graph<Graph, EdgeIndexMap>
      eig(g, get(&EdgeProperties::index, g));
   
   NN nnlin(eig, get(&VertexProperties::state,g), space);
   
   // roadmapgen
   RoadmapArgs rm_args(space, eig,
      get(&VertexProperties::state, g),
      get(&EdgeProperties::distance, g),
      get(&VertexProperties::subgraph, g),
      get(&EdgeProperties::subgraph, g),
      get(&VertexProperties::is_shadow, g),
      eig.edge_vector_map,
      &nnlin);
   RoadmapPtr p_mygen(new ompl_lemur::RoadmapHalton<RoadmapArgs>(rm_args));
   p_mygen->setNum(30);
   p_mygen->setRadius(0.3);

   // generate a graph
   p_mygen->initialize();
   p_mygen->generate();

   {
      // write it out to file
      boost::dynamic_properties props;
      props.property("state", ompl_lemur::make_rvstate_map_string_adaptor(
         get(&VertexProperties::state,g),
         space->as<ompl::base::RealVectorStateSpace>()));
      props.property("subgraph", pr_bgl::make_string_map(get(&VertexProperties::subgraph,g)));
      props.property("subgraph", pr_bgl::make_string_map(get(&EdgeProperties::subgraph,g)));
      props.property("is_shadow", pr_bgl::make_string_map(get(&VertexProperties::is_shadow,g)));
      props.property("distance", pr_bgl::make_string_map(get(&EdgeProperties::distance,g)));
      
      printf("original graph:\n");
      pr_bgl::write_graphio_graph(std::cout, g,
         get(boost::vertex_index, g), get(&EdgeProperties::index, g));
      pr_bgl::write_graphio_properties(std::cout, g,
         get(boost::vertex_index, g), get(&EdgeProperties::index, g),
         props);
   }
   
   OverGraph og;

   pr_bgl::overlay_manager<
         EdgeIndexedGraph,
         OverGraph,
         boost::property_map<OverGraph, Vertex OverVertexProperties::*>::type,
         boost::property_map<OverGraph, Edge OverEdgeProperties::*>::type
         >
      overlay_manager(
         eig,
         og,
         get(&OverVertexProperties::core_vertex, og),
         get(&OverEdgeProperties::core_edge, og)
         );

   // add some roots to the overlay graph
   {
      // the root itself (doesnt correspond to a core vertex)
      OverVertex v_root = add_vertex(og);
      og[v_root].core_vertex = boost::graph_traits<Graph>::null_vertex();
      // set state
      og[v_root].core_properties.state = space->allocState();
      ompl::base::State * v_state = og[v_root].core_properties.state;
      double * values = v_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      values[0] = 0.2;
      values[1] = 0.2;
      og[v_root].core_properties.subgraph = 0;
      og[v_root].core_properties.is_shadow = false;
      
      // find closest core vertex
      Vertex v_closest = boost::graph_traits<Graph>::null_vertex();
      double dist_closest = HUGE_VAL;
      VertexIter vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      {
         double dist = space->distance(
            og[v_root].core_properties.state,
            g[*vi].state);
         if (dist < dist_closest)
         {
            dist_closest = dist;
            v_closest = *vi;
         }
      }
      
      // add new anchor overlay vertex
      OverVertex v_anchor= add_vertex(og);
      og[v_anchor].core_vertex = v_closest;
      // no need to set core properties (e.g. state) on anchors,
      // since this is just an anchor, wont be copied

      // add overlay edge from root to anchor
      OverEdge e = add_edge(v_root, v_anchor, og).first;
      // add edge properties
      // og[e].core_properties.index -- needs to be set on apply
      og[e].core_properties.distance = dist_closest;
      og[e].core_properties.subgraph = 0;
   }

   // ok, APPLY!
   overlay_manager.apply();
   
   // manually copy over properties
   for (unsigned int ui=0; ui<overlay_manager.applied_vertices.size(); ui++)
   {
      OverVertex vover = overlay_manager.applied_vertices[ui];
      Vertex vcore = og[vover].core_vertex;
      g[vcore] = og[vover].core_properties;
   }
   for (unsigned int ui=0; ui<overlay_manager.applied_edges.size(); ui++)
   {
      OverEdge eover = overlay_manager.applied_edges[ui];
      Edge ecore = og[eover].core_edge;
      g[ecore].distance = og[eover].core_properties.distance;
      g[ecore].subgraph = og[eover].core_properties.subgraph;
      // IMPORTANT: DONT COPY OVER EDGE INDEX, ITS NOT RIGHT IN OVERLAY!
   }
   
   {
      // write it out to file
      boost::dynamic_properties props;
      props.property("state", ompl_lemur::make_rvstate_map_string_adaptor(
         get(&VertexProperties::state,g),
         space->as<ompl::base::RealVectorStateSpace>()));
      props.property("subgraph", pr_bgl::make_string_map(get(&VertexProperties::subgraph,g)));
      props.property("subgraph", pr_bgl::make_string_map(get(&EdgeProperties::subgraph,g)));
      props.property("is_shadow", pr_bgl::make_string_map(get(&VertexProperties::is_shadow,g)));
      props.property("distance", pr_bgl::make_string_map(get(&EdgeProperties::distance,g)));
      printf("overlayed graph:\n");
      pr_bgl::write_graphio_graph(std::cout, g,
         get(boost::vertex_index, g), get(&EdgeProperties::index, g));
      pr_bgl::write_graphio_properties(std::cout, g,
         get(boost::vertex_index, g), get(&EdgeProperties::index, g),
         props);
   }

   overlay_manager.unapply();
   
   {
      // write it out to file
      boost::dynamic_properties props;
      props.property("state", ompl_lemur::make_rvstate_map_string_adaptor(
         get(&VertexProperties::state,g),
         space->as<ompl::base::RealVectorStateSpace>()));
      props.property("subgraph", pr_bgl::make_string_map(get(&VertexProperties::subgraph,g)));
      props.property("subgraph", pr_bgl::make_string_map(get(&EdgeProperties::subgraph,g)));
      props.property("is_shadow", pr_bgl::make_string_map(get(&VertexProperties::is_shadow,g)));
      props.property("distance", pr_bgl::make_string_map(get(&EdgeProperties::distance,g)));
      printf("unapplied graph:\n");
      pr_bgl::write_graphio_graph(std::cout, g,
         get(boost::vertex_index, g), get(&EdgeProperties::index, g));
      pr_bgl::write_graphio_properties(std::cout, g,
         get(boost::vertex_index, g), get(&EdgeProperties::index, g),
         props);
   }
   
   return 0;
}
