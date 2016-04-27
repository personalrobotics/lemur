/*! \file generate_roadmap.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
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
#include <pr_bgl/vector_ref_property_map.h>
#include <pr_bgl/edge_indexed_graph.h>

#include <ompl_lemur/util.h>
#include <ompl_lemur/rvstate_map_string_adaptor.h>
#include <ompl_lemur/FnString.h>
#include <ompl_lemur/SamplerGenMonkeyPatch.h>
#include <ompl_lemur/NearestNeighborsLinearBGL.h>
#include <ompl_lemur/Roadmap.h>
#include <ompl_lemur/RoadmapAAGrid.h>
#include <ompl_lemur/RoadmapFromFile.h>
#include <ompl_lemur/RoadmapHalton.h>
#include <ompl_lemur/RoadmapHaltonDens.h>
#include <ompl_lemur/RoadmapHaltonOffDens.h>
#include <ompl_lemur/RoadmapHaltonOffLLDens.h>
#include <ompl_lemur/RoadmapRGG.h>
#include <ompl_lemur/RoadmapRGGDens.h>
#include <ompl_lemur/RoadmapRGGDensConst.h>

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

typedef boost::property_map<Graph, ompl::base::State * VertexProperties::*>::type StateMap;
typedef boost::property_map<Graph, int VertexProperties::*>::type VertexBatchMap;
typedef boost::property_map<Graph, int EdgeProperties::*>::type EdgeBatchMap;
typedef boost::property_map<Graph, bool VertexProperties::*>::type IsShadowMap;
typedef boost::property_map<Graph, double EdgeProperties::*>::type DistanceMap;

typedef pr_bgl::edge_indexed_graph<Graph, EdgeIndexMap> EdgeIndexedGraph;
typedef ompl_lemur::NearestNeighborsLinearBGL<EdgeIndexedGraph,StateMap> NN;

typedef ompl_lemur::RoadmapArgs<EdgeIndexedGraph,StateMap,DistanceMap,VertexBatchMap,EdgeBatchMap,IsShadowMap,EdgeIndexedGraph::EdgeVectorMap,NN> RoadmapArgs;
typedef boost::shared_ptr< ompl_lemur::Roadmap<RoadmapArgs> > RoadmapPtr;


int main(int argc, char **argv)
{
   boost::program_options::options_description desc("Allowed options");
   desc.add_options()
      ("help", "produce help message")
      ("dim", boost::program_options::value<int>(), "hypercube dimension (e.g. 2)")
      ("bounds", boost::program_options::value< std::vector<std::string> >(), "(e.g. 0:-2:2)")
      ("roadmap-type", boost::program_options::value<std::string>(), "(e.g. Halton)")
      ("roadmap-param", boost::program_options::value< std::vector<std::string> >(), "(e.g. num=30)")
      ("num-batches", boost::program_options::value<std::size_t>(), "number of batches (e.g. 1)")
      ("out-file", boost::program_options::value<std::string>(), "output file (can be - for stdout)")
      ("out-format", boost::program_options::value<std::string>(), "output format (graphml or graphio)")
   ;
   
   boost::program_options::variables_map args;
   boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), args);
   boost::program_options::notify(args);    

   if (args.count("help"))
   {
      std::cout << desc << std::endl;
      return 0;
   }
   
   const char * reqs[] = {"dim", "roadmap-type", "num-batches", "out-file", "out-format"};
   for (unsigned int ui=0; ui<sizeof(reqs)/sizeof(reqs[0]); ui++)
   {
      if (args.count(reqs[ui]) != 1)
      {
         OMPL_ERROR("--%s must be passed.", reqs[ui]);
         return 1;
      }
   }
   
   int dim = args["dim"].as<int>();
   OMPL_INFORM("Creating unit ompl space of dimension %d ...", dim);
   ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dim));
   ompl::base::RealVectorBounds space_bounds(dim);
   space_bounds.setLow(0.0);
   space_bounds.setHigh(1.0);
   
   if (args.count("bounds"))
   {
      const std::vector<std::string> & bounds = args["bounds"].as< std::vector<std::string> >();
      
      for (unsigned int ui=0; ui<bounds.size(); ui++)
      {
         int idof;
         double lower;
         double upper;
         int n;
         int ret;
         ret = sscanf(bounds[ui].c_str(), "%d:%lf,%lf%n", &idof, &lower, &upper, &n);
         if (ret != 3 || n != (int)bounds[ui].size() || idof<0 || dim<=idof)
         {
            OMPL_ERROR("--bounds %s not properly formatted.", bounds[ui].c_str());
            return 1;
         }
         space_bounds.setLow(idof, lower);
         space_bounds.setHigh(idof, upper);
      }
   }
   
   space->as<ompl::base::RealVectorStateSpace>()->setBounds(space_bounds);
   
   RoadmapPtr p_mygen;
   
   Graph g;
   
   pr_bgl::edge_indexed_graph<Graph, EdgeIndexMap>
      eig(g, get(&EdgeProperties::index, g));
   
   NN nnlin(eig, get(&VertexProperties::state,g), space);
   
   // construct roadmap
   RoadmapArgs rmargs(space, eig, 
      get(&VertexProperties::state, g),
      get(&EdgeProperties::distance, g),
      get(&VertexProperties::batch, g),
      get(&EdgeProperties::batch, g),
      get(&VertexProperties::is_shadow, g),
      eig.edge_vector_map,
      &nnlin);
   
   if (args["roadmap-type"].as<std::string>() == "AAGrid")
      p_mygen.reset(new ompl_lemur::RoadmapAAGrid<RoadmapArgs>(rmargs));
   else if (args["roadmap-type"].as<std::string>() == "FromFile")
      p_mygen.reset(new ompl_lemur::RoadmapFromFile<RoadmapArgs>(rmargs));
   else if (args["roadmap-type"].as<std::string>() == "RGG")
      p_mygen.reset(new ompl_lemur::RoadmapRGG<RoadmapArgs>(rmargs));
   else if (args["roadmap-type"].as<std::string>() == "RGGDens")
      p_mygen.reset(new ompl_lemur::RoadmapRGGDens<RoadmapArgs>(rmargs));
   else if (args["roadmap-type"].as<std::string>() == "RGGDensConst")
      p_mygen.reset(new ompl_lemur::RoadmapRGGDensConst<RoadmapArgs>(rmargs));
   else if (args["roadmap-type"].as<std::string>() == "Halton")
      p_mygen.reset(new ompl_lemur::RoadmapHalton<RoadmapArgs>(rmargs));
   else if (args["roadmap-type"].as<std::string>() == "HaltonDens")
      p_mygen.reset(new ompl_lemur::RoadmapHaltonDens<RoadmapArgs>(rmargs));
   else if (args["roadmap-type"].as<std::string>() == "HaltonOffDens")
      p_mygen.reset(new ompl_lemur::RoadmapHaltonOffDens<RoadmapArgs>(rmargs));
   else if (args["roadmap-type"].as<std::string>() == "HaltonOffLLDens")
      p_mygen.reset(new ompl_lemur::RoadmapHaltonOffLLDens<RoadmapArgs>(rmargs));
   else
   {
      OMPL_ERROR("--roadmap-type unknown!");
      return 1;
   }
   
   if (args.count("roadmap-param"))
   {
      const std::vector<std::string> & params = args["roadmap-param"].as< std::vector<std::string> >();
      for (unsigned int ui=0; ui<params.size(); ui++)
      {
         size_t eq = params[ui].find('=');
         if (eq == params[ui].npos)
         {
            OMPL_ERROR("--roadmap-param has bad format!");
            return 1;
         }
         p_mygen->params.setParam(params[ui].substr(0,eq), params[ui].substr(eq+1));
      }
   }
   
   std::size_t num_batches = args["num-batches"].as<std::size_t>();
   OMPL_INFORM("Generating %lu batch%s ...", num_batches, num_batches==1?"":"es");
   
   p_mygen->initialize();
   while (p_mygen->num_batches_generated < num_batches)
   {
      // generate a graph
      p_mygen->generate();
   }
   
   OMPL_INFORM("Generated graph has %lu vertices and %lu edges.", num_vertices(g), num_edges(g));
   
   // write it out to file
   boost::dynamic_properties props;
   props.property("state", ompl_lemur::make_rvstate_map_string_adaptor(
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
      if (!fp.is_open())
      {
         OMPL_ERROR("could not open file!");
         return 1;
      }
      outp = &fp;
   }
   
   std::string out_format = args["out-format"].as<std::string>();
   if (out_format == "graphio")
   {
      OMPL_INFORM("Writing to graphio file ...");
      pr_bgl::write_graphio_graph(*outp, g,
         get(boost::vertex_index,g), get(&EdgeProperties::index,g));
      pr_bgl::write_graphio_properties(*outp, g,
         get(boost::vertex_index,g), get(&EdgeProperties::index,g),
         props);
   }
   else if (out_format == "graphml")
   {
      OMPL_INFORM("Writing to graphml file ...");
      boost::write_graphml(*outp, g, get(boost::vertex_index,g), props, true); // ordered_vertices
   }
   else
   {
      OMPL_ERROR("--out-format must be graphio or graphml!");
      return 1;
   }
   
   if (fp.is_open())
      fp.close();
   
   return 0;
}
