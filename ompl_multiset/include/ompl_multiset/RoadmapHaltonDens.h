/* File: RoadmapHalton.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
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
class RoadmapHaltonDens : public RoadmapSpec
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
   
   RoadmapHaltonDens(
      const ompl::base::StateSpacePtr space,
      unsigned int n_perbatch, double radius_firstbatch):
      RoadmapSpec(space,0),
      n_perbatch(n_perbatch), radius_firstbatch(radius_firstbatch),
      dim(0),
      bounds(0)
   {
      // check that we're in a real vector state space
      if (space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapHaltonDens only supports rel vector state spaces!");
      dim = space->getDimension();
      if (0 == ompl_multiset::util::get_prime(dim-1))
         throw std::runtime_error("not enough primes hardcoded!");
      bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
   }
   ~RoadmapHaltonDens() {}
   
   double root_radius(std::size_t i_batch)
   {
      return radius_firstbatch * pow(1./(i_batch+1.), 1./dim);
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
      double radius = root_radius(this->num_batches_generated);
      std::size_t n = (this->num_batches_generated+1) * n_perbatch;
      for (std::size_t v_index=num_vertices(g); v_index<n; v_index++)
      {
         Vertex v_new = add_vertex(g);
         
         put(vertex_batch_map, v_new, this->num_batches_generated);
         put(is_shadow_map, v_new, false);
         
         // allocate a new state for this vertex
         ompl::base::State * v_state = this->space->allocState();
         put(state_map, v_new, v_state);
         double * values = v_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
         for (unsigned int ui=0; ui<dim; ui++)
            values[ui] = bounds.low[ui] + (bounds.high[ui] - bounds.low[ui])
               * ompl_multiset::util::halton(
                  ompl_multiset::util::get_prime(ui), v_index);
         nn->add(v_new);

         // allocate new undirected edges
         std::vector<Vertex> vs_near;
         nn->nearestR(v_new, radius, vs_near);
         for (unsigned int ui=0; ui<vs_near.size(); ui++)
         {
            Edge e = add_edge(v_new, vs_near[ui], g).first;
            ompl::base::State * vnear_state = get(state_map,vs_near[ui]);
            put(distance_map, e, this->space->distance(v_state,vnear_state));
            put(edge_batch_map, e, this->num_batches_generated);
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
   // from space
   unsigned int dim;
   ompl::base::RealVectorBounds bounds;
};

} // namespace ompl_multiset
