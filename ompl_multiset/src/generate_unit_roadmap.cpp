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
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <pr_bgl/graph_io.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/edge_indexed_graph.h>

#include <ompl_multiset/util.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/RoadmapGen.h>
#include <ompl_multiset/RoadmapGenAAGrid.h>
#include <ompl_multiset/RoadmapGenHalton.h>
#include <ompl_multiset/RoadmapGenHaltonDens.h>
#include <ompl_multiset/RoadmapGenRGG.h>


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

typedef boost::property_map<Graph, StateContainerPtr VertexProperties::*>::type StateMap;
typedef boost::property_map<Graph, int VertexProperties::*>::type VertexBatchMap;
typedef boost::property_map<Graph, int EdgeProperties::*>::type EdgeBatchMap;
typedef boost::property_map<Graph, bool VertexProperties::*>::type IsShadowMap;
typedef boost::property_map<Graph, double EdgeProperties::*>::type DistanceMap;

typedef pr_bgl::EdgeIndexedGraph<Graph, EdgeIndexMap> EdgeIndexedGraph;
typedef ompl_multiset::RoadmapGen<EdgeIndexedGraph,StateMap,DistanceMap,VertexBatchMap,EdgeBatchMap,IsShadowMap> RoadmapGen;
typedef boost::shared_ptr<RoadmapGen> RoadmapGenPtr;


inline void stringify_from_x(std::string & repr, const StateContainerPtr & in)
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

inline void stringify_to_x(const std::string & in, StateContainerPtr & repr)
{
   repr.reset();
   //repr = atof(in.c_str());
}


int main(int argc, char **argv)
{
   boost::program_options::options_description desc("Allowed options");
   desc.add_options()
      ("help", "produce help message")
      ("dim", boost::program_options::value<int>(), "unit hypercube dimension (e.g. 2)")
      ("roadmap-type", boost::program_options::value<std::string>(), "(e.g. Halton)")
      ("roadmap-args", boost::program_options::value<std::string>(), "(e.g. 'n=30 radius=0.3')")
      ("num-batches", boost::program_options::value<int>(), "number of batches (e.g. 1)")
      ("out-file", boost::program_options::value<std::string>(), "output file (can be - for stdout)")
      ("out-format", boost::program_options::value<std::string>(), "output format (graphml or graphio)")
   ;
   
   boost::program_options::variables_map args;
   boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), args);
   boost::program_options::notify(args);    

   if (args.count("help")
      || args.count("dim") != 1
      || args.count("roadmap-type") != 1
      || args.count("roadmap-args") != 1
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
   
   RoadmapGenPtr p_mygen;
   
   std::string roadmap_type(args["roadmap-type"].as<std::string>());
   std::transform(roadmap_type.begin(), roadmap_type.end(), roadmap_type.begin(), ::tolower);
   printf("creating roadmap of type %s ...\n", roadmap_type.c_str());
   if (roadmap_type == "aagrid")
      p_mygen.reset(new ompl_multiset::RoadmapGenAAGrid<RoadmapGen>(space, args["roadmap-args"].as<std::string>()));
   else if (roadmap_type == "rgg")
      p_mygen.reset(new ompl_multiset::RoadmapGenRGG<RoadmapGen>(space, args["roadmap-args"].as<std::string>()));
   else if (roadmap_type == "halton")
      p_mygen.reset(new ompl_multiset::RoadmapGenHalton<RoadmapGen>(space, args["roadmap-args"].as<std::string>()));
   else if (roadmap_type == "haltondens")
      p_mygen.reset(new ompl_multiset::RoadmapGenHaltonDens<RoadmapGen>(space, args["roadmap-args"].as<std::string>()));
   else
   {
      printf("unknown roadmap type!\n");
      return 1;
   }
   
   Graph g;
   
   pr_bgl::EdgeIndexedGraph<Graph, EdgeIndexMap>
      eig(g, get(&EdgeProperties::index, g));
   
   
   std::size_t num_batches = args["num-batches"].as<unsigned int>();
   printf("generating %lu batches ...\n", num_batches);
   
   while (p_mygen->get_num_batches_generated() < num_batches)
   {
      // generate a graph
      p_mygen->generate(eig,
         get(&VertexProperties::state, g),
         get(&EdgeProperties::distance, g),
         get(&VertexProperties::batch, g),
         get(&EdgeProperties::batch, g),
         get(&VertexProperties::is_shadow, g));
   }
   
   // write it out to file
   boost::dynamic_properties props;
   props.property("state", pr_bgl::make_string_map(get(&VertexProperties::state,g)));
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
      write_graphml(*outp, g, get(boost::vertex_index,g), props, true); // ordered_vertices
   }
   else
      throw std::runtime_error("out format must be graphio or graphml!");
   
   if (fp.is_open())
      fp.close();
   
   return 0;
}
