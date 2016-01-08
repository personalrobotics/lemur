/* File: generate_unit_roadmap.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <algorithm>
#include <fstream>

#include <boost/property_map/property_map.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/program_options.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include <pr_bgl/graph_io.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/edge_indexed_graph.h>

#include <ompl_multiset/util.h>
#include <ompl_multiset/rvstate_map_string_adaptor.h>
#include <ompl_multiset/FnString.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/NearestNeighborsLinearBGL.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapAAGrid.h>
#include <ompl_multiset/RoadmapFromFile.h>
#include <ompl_multiset/RoadmapHalton.h>
#include <ompl_multiset/RoadmapHaltonDens.h>
#include <ompl_multiset/RoadmapHaltonOffDens.h>
#include <ompl_multiset/RoadmapRGG.h>
#include <ompl_multiset/RoadmapRGGDens.h>
#include <ompl_multiset/RoadmapRGGDensConst.h>
#include <ompl_multiset/RoadmapID.h>

struct VertexProperties
{
   ompl::base::State * state;
   int batch;
   bool is_shadow;
};
struct EdgeProperties
{
   std::size_t index;
   double distance;
   int batch;
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

typedef boost::property_map<Graph, ompl::base::State * VertexProperties::*>::type StateMap;
typedef boost::property_map<Graph, int VertexProperties::*>::type VertexBatchMap;
typedef boost::property_map<Graph, int EdgeProperties::*>::type EdgeBatchMap;
typedef boost::property_map<Graph, bool VertexProperties::*>::type IsShadowMap;
typedef boost::property_map<Graph, double EdgeProperties::*>::type DistanceMap;

typedef pr_bgl::EdgeIndexedGraph<Graph, EdgeIndexMap> EdgeIndexedGraph;
typedef ompl_multiset::NearestNeighborsLinearBGL<Graph,StateMap> NN;
typedef ompl_multiset::Roadmap<EdgeIndexedGraph,StateMap,DistanceMap,VertexBatchMap,EdgeBatchMap,IsShadowMap,NN> Roadmap;
typedef boost::shared_ptr<Roadmap> RoadmapPtr;


int main(int argc, char **argv)
{
   boost::program_options::options_description desc("Allowed options");
   desc.add_options()
      ("help", "produce help message")
      ("dim", boost::program_options::value<int>(), "unit hypercube dimension (e.g. 2)")
      ("roadmap-id", boost::program_options::value<std::string>(), "(e.g. Halton(n=30 radius=0.3))")
      ("num-batches", boost::program_options::value<std::size_t>(), "number of batches (e.g. 1)")
      ("out-file", boost::program_options::value<std::string>(), "output file (can be - for stdout)")
      ("out-format", boost::program_options::value<std::string>(), "output format (graphml or graphio)")
   ;
   
   boost::program_options::variables_map args;
   boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), args);
   boost::program_options::notify(args);    

   if (args.count("help")
      || args.count("dim") != 1
      || args.count("roadmap-id") != 1
      || args.count("num-batches") != 1
      || args.count("out-file") != 1
      || args.count("out-format") != 1)
   {
      std::cout << desc << std::endl;
      return 1;
   }
   
   int dim = args["dim"].as<int>();
   printf("creating unit ompl space of dimension %d ...\n", dim);
   ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dim));
   space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
   
   RoadmapPtr p_mygen;
   
   std::string roadmap_id(args["roadmap-id"].as<std::string>());
   printf("creating roadmap with id %s ...\n", roadmap_id.c_str());
   p_mygen.reset(ompl_multiset::make_roadmap_gen<Roadmap>(space, roadmap_id));
   
   Graph g;
   
   pr_bgl::EdgeIndexedGraph<Graph, EdgeIndexMap>
      eig(g, get(&EdgeProperties::index, g));
   
   std::size_t num_batches = args["num-batches"].as<std::size_t>();
   printf("generating %lu batches ...\n", num_batches);
   
   while (p_mygen->get_num_batches_generated() < num_batches)
   {
      // generate a graph
      NN nnlin(g, get(&VertexProperties::state,g), space);
      p_mygen->generate(eig, &nnlin,
         get(&VertexProperties::state, g),
         get(&EdgeProperties::distance, g),
         get(&VertexProperties::batch, g),
         get(&EdgeProperties::batch, g),
         get(&VertexProperties::is_shadow, g));
   }
   
   // write it out to file
   boost::dynamic_properties props;
   props.property("state", ompl_multiset::make_rvstate_map_string_adaptor(
      get(&VertexProperties::state,g),
      space->as<ompl::base::RealVectorStateSpace>()));
   props.property("batch", pr_bgl::make_string_map(get(&VertexProperties::batch,g)));
   props.property("batch", pr_bgl::make_string_map(get(&EdgeProperties::batch,g)));
   props.property("is_shadow", pr_bgl::make_string_map(get(&VertexProperties::is_shadow,g)));
   props.property("distance", pr_bgl::make_string_map(get(&EdgeProperties::distance,g)));
   
   std::ostream * outp;
   std::ofstream fp;
   std::string out_file = args["out-file"].as<std::string>();
   if (out_file == "-")
      outp = &std::cout;
   else
   {
      fp.open(out_file.c_str());
      assert(fp.is_open());
      outp = &fp;
   }
   
   std::string out_format = args["out-format"].as<std::string>();
   if (out_format == "graphio")
   {
      pr_bgl::write_graphio_graph(*outp, g,
         get(boost::vertex_index,g), get(&EdgeProperties::index,g));
      pr_bgl::write_graphio_properties(*outp, g,
         get(boost::vertex_index,g), get(&EdgeProperties::index,g),
         props);
   }
   else if (out_format == "graphml")
   {
      boost::write_graphml(*outp, g, get(boost::vertex_index,g), props, true); // ordered_vertices
   }
   else
      throw std::runtime_error("out format must be graphio or graphml!");
   
   if (fp.is_open())
      fp.close();
   
   return 0;
}
