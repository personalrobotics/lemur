/* File: GenerateFromRoadmap.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

/* requires:
#include <ompl/datastructures/NearestNeighborsLinear.h>
#include <pr_bgl/throw_map.h>
*/

namespace ompl_lemur
{

template <typename Vertex, class VState>
double generate_from_roadmap_dist(ompl::base::StateSpacePtr space, VState vstate_map, Vertex va, Vertex vb)
{
   return space->distance(get(vstate_map,va), get(vstate_map,vb));
}

// here's some code which just generates one batch of a roadmap into a graph
template <template<class> class RoadmapTemplate,
   class Graph, class VState, class EDistance, class VBatch, class EBatch, class VShadow>
void generate_from_roadmap(
   ompl::base::StateSpacePtr space,
   Graph & g,
   VState vstate_map,
   EDistance edistance_map,
   VBatch vbatch_map,
   EBatch ebatch_map,
   VShadow vshadow_map,
   const std::map<std::string, std::string> & roadmap_params,
   size_t num_batches)
{
   typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
   typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
   
   // construct a simple (temporary) linear nearest neighbor object
   typedef ompl::NearestNeighborsLinear<Vertex> NN;
   NN mynn;
   mynn.setDistanceFunction(boost::bind(generate_from_roadmap_dist<Vertex,VState>, space, vstate_map, _1, _2));
   
   // construct bogus edge vector map
   typedef pr_bgl::throw_map<size_t, Edge> EVector;
   EVector evector_map;
   
   // construct a RoadmapArgs object
   typedef ompl_lemur::RoadmapArgs<Graph,VState,EDistance,VBatch,EBatch,VShadow,EVector,NN> RoadmapArgs;
   RoadmapArgs args(space, g, vstate_map, edistance_map, vbatch_map, ebatch_map, vshadow_map, evector_map, &mynn);
   
   // construct a Roadmap object
   RoadmapTemplate<RoadmapArgs> roadmap(args);
   
   // set its parameters
   for (std::map<std::string, std::string>::const_iterator
      it=roadmap_params.begin(); it!=roadmap_params.end(); it++)
   {
      roadmap.params.setParam(it->first, it->second);
   }
   
   // initialize roadmap
   roadmap.initialize();
   
   // generate
   for (size_t ibatch=0; ibatch<num_batches; ibatch++)
      roadmap.generate();
}

} // namespace ompl_lemur
