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
   //class StateMap, class BatchMap, class IsShadowMap, class DistanceMap
//   >
template <class RoadmapSpec>
class RoadmapRGG : public RoadmapSpec
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
   const unsigned int n;
   const double radius;
   const unsigned int seed;
   
   RoadmapRGG(
      const ompl::base::StateSpacePtr space,
      unsigned int n, double radius, unsigned int seed):
      RoadmapSpec(space,1),
      n(n), radius(radius), seed(seed),
      sampler(space->allocStateSampler())
   {
      ompl_multiset::SamplerGenMonkeyPatch(sampler) = boost::mt19937(seed);
   }
   ~RoadmapRGG() {}
   
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
      if (this->max_batches < this->num_batches_generated + 1)
         throw std::runtime_error("this roadmap gen doesnt support that many batches!");
      // ok, generate n nodes!
      for (std::size_t v_index=num_vertices(g); v_index<n; v_index++)
      {
         Vertex v_new = add_vertex(g);
         
         put(vertex_batch_map, v_new, 0);
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
            put(edge_batch_map, e, 0);
         }
      }
      this->num_batches_generated++;
   }
   
   void serialize()
   {
   }
   
   void deserialize()
   {
   }
   
private:
   ompl::base::StateSamplerPtr sampler;
};

} // namespace ompl_multiset
