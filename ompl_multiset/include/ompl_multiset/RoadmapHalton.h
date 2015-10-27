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
class RoadmapHalton : public RoadmapSpec
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

   RoadmapHalton(
      const ompl::base::StateSpacePtr space,
      unsigned int n, double radius):
      RoadmapSpec(space,1),
      n(n), radius(radius),
      dim(0),
      bounds(0),
      num_batches_generated(0),
      vertices_generated(0),
      edges_generated(0)
   {
      // check that we're in a real vector state space
      if (space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapHalton only supports rel vector state spaces!");
      dim = space->getDimension();
      if (0 == ompl_multiset::util::get_prime(dim-1))
         throw std::runtime_error("not enough primes hardcoded!");
      bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
   }
   ~RoadmapHalton() {}
   
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
      NN & nn,
      VState state_map,
      EDistance distance_map,
      VBatch vertex_batch_map,
      EBatch edge_batch_map,
      VShadow is_shadow_map)
   {
      if (this->max_batches < num_batches_generated + 1)
         throw std::runtime_error("this roadmap gen doesnt support that many batches!");
      // ok, generate n nodes!
      while (num_vertices(g) < n)
      {
         Vertex v_new = add_vertex(g);
         
         put(vertex_batch_map, v_new, 0);
         put(is_shadow_map, v_new, false);
         
         // allocate a new state for this vertex
         ompl::base::State * v_state = this->space->allocState();
         put(state_map, v_new, v_state);
         double * values = v_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
         for (unsigned int ui=0; ui<dim; ui++)
            values[ui] = bounds.low[ui] + (bounds.high[ui] - bounds.low[ui])
               * ompl_multiset::util::halton(
                  ompl_multiset::util::get_prime(ui), vertices_generated);
         
         // allocate new undirected edges
         std::vector< std::pair<Vertex,double> > vs_near;
         nn.nearestR(v_new, radius, vs_near);
         for (unsigned int ui=0; ui<vs_near.size(); ui++)
         {
            Edge e = add_edge(v_new, vs_near[ui].first, g).first;
            put(distance_map, e, vs_near[ui].second);
            put(edge_batch_map, e, 0);
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
   ompl::base::RealVectorBounds bounds;
   // from id
  
   // progress
   std::size_t num_batches_generated;
   std::size_t vertices_generated;
   std::size_t edges_generated;
};

} // namespace ompl_multiset
