/* File: RoadmapRGGDens.h
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
class RoadmapRGGDens : public RoadmapSpec
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
   const double radius_firstbatch;
   const unsigned int seed;

   RoadmapRGGDens(
      const ompl::base::StateSpacePtr space,
      unsigned int n_perbatch, double radius_firstbatch, unsigned int seed):
      RoadmapSpec(space,0),
      n_perbatch(n_perbatch), radius_firstbatch(radius_firstbatch), seed(seed),
      num_batches_generated(0),
      vertices_generated(0),
      edges_generated(0),
      sampler(space->allocStateSampler())
   {
      // check that we're in a real vector state space
      if (space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapRGGDens only supports rel vector state spaces!");
      dim = space->getDimension();
      gamma = radius_firstbatch / pow(log(n_perbatch)/n_perbatch, 1./dim);
      printf("RoadmapRGGDens calculated gamma=%f\n", gamma);
      ompl_multiset::SamplerGenMonkeyPatch(sampler) = boost::mt19937(seed);
   }
   ~RoadmapRGGDens() {}
   
   std::size_t get_num_batches_generated()
   {
      return num_batches_generated;
   }
   
   double root_radius(std::size_t i_batch)
   {
      unsigned int n = (1 + i_batch) * n_perbatch;
      return gamma * pow(log(n)/n, 1./dim);
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
      // compute radius
      double radius = root_radius(num_batches_generated);
      while (num_vertices(g) < (num_batches_generated+1) * n_perbatch)
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
   // from space
   unsigned int dim;
   double gamma;
   // progress
   std::size_t num_batches_generated;
   std::size_t vertices_generated;
   std::size_t edges_generated;
   ompl::base::StateSamplerPtr sampler;
};

} // namespace ompl_multiset
