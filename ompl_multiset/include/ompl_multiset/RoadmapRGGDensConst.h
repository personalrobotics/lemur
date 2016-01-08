/* File: RoadmapRGG.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

/* requires:
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
*/

namespace ompl_multiset
{

// for now this is an r-disk prm,
// uniform milestone sampling with given seed,
// uses the space's default sampler
//template <class Graph, class VertexIndexMap, class EdgeIndexMap//,
   //class StateMap, class BatcMap, class IsShadowMap, class DistanceMap
//   >
template <class RoadmapSpec>
class RoadmapRGGDensConst : public RoadmapSpec
{
   typedef typename RoadmapSpec::BaseGraph Graph;
   typedef typename RoadmapSpec::BaseVState VState;
   typedef typename RoadmapSpec::BaseEDistance EDistance;
   typedef typename RoadmapSpec::BaseVBatch VBatch;
   typedef typename RoadmapSpec::BaseEBatch EBatch;
   typedef typename RoadmapSpec::BaseVShadow VShadow;
   typedef typename RoadmapSpec::BaseNN NN;
   
   typedef boost::graph_traits<Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   typedef typename GraphTypes::edge_descriptor Edge;
   
public:
   // input parameters
   const unsigned int n_perbatch;
   const double radius;
   const unsigned int seed;

   RoadmapRGGDensConst(
      const ompl::base::StateSpacePtr space,
      unsigned int n_perbatch, double radius, unsigned int seed):
      RoadmapSpec(space,0),
      n_perbatch(n_perbatch), radius(radius), seed(seed),
      num_batches_generated(0),
      vertices_generated(0),
      edges_generated(0),
      sampler(space->allocStateSampler())
   {
      ompl_multiset::SamplerGenMonkeyPatch(sampler) = boost::mt19937(seed);
   }
   ~RoadmapRGGDensConst() {}
   
   std::size_t get_num_batches_generated()
   {
      return num_batches_generated;
   }
   
   double root_radius(std::size_t i_batch)
   {
      return radius;
   }
   
   void generate(
      Graph & g,
      NN * nn,
      VState state_map,
      EDistance distance_map,
      VBatch vertex_batch_map,
      EBatch edge_batch_map,
      VShadow is_shadow_map)
   {
      // ok, generate n nodes!
      while (num_vertices(g) < (num_batches_generated+1)*n_perbatch)
      {
         Vertex v_new = add_vertex(g);
         
         put(vertex_batch_map, v_new, num_batches_generated);
         put(is_shadow_map, v_new, false);
         
         // allocate a new state for this vertex
         ompl::base::State * v_state = this->space->allocState();
         this->sampler->sampleUniform(v_state);
         put(state_map, v_new, v_state);
         nn->add(v_new);
         
         // allocate new undirected edges
         std::vector<Vertex> vs_near;
         nn->nearestR(v_new, radius, vs_near);
         for (unsigned int ui=0; ui<vs_near.size(); ui++)
         {
            Edge e = add_edge(v_new, vs_near[ui], g).first;
            ompl::base::State * vnear_state = get(state_map, vs_near[ui]);
            put(distance_map, e, this->space->distance(v_state,vnear_state));
            put(edge_batch_map, e, num_batches_generated);
            edges_generated++;
         }
         
         vertices_generated++;
      }
      num_batches_generated++;
   }
   
   void serialize()
   {
   }
   
   void deserialize()
   {
   }
   
private:
   // progress
   std::size_t num_batches_generated;
   std::size_t vertices_generated;
   std::size_t edges_generated;
   ompl::base::StateSamplerPtr sampler;
};

} // namespace ompl_multiset
