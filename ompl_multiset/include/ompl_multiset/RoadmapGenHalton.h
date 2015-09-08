/* File: RoadmapGenHalton.h
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
   //class StateMap, class SubgraphMap, class IsShadowMap, class DistanceMap
//   >
template <class RoadmapGenSpec>
class RoadmapGenHalton : public RoadmapGenSpec
{
   typedef typename RoadmapGenSpec::BaseGraph Graph;
   typedef typename RoadmapGenSpec::BaseVState VState;
   typedef typename RoadmapGenSpec::BaseEDistance EDistance;
   typedef typename RoadmapGenSpec::BaseVSubgraph VSubgraph;
   typedef typename RoadmapGenSpec::BaseESubgraph ESubgraph;
   typedef typename RoadmapGenSpec::BaseVShadow VShadow;

   typedef boost::graph_traits<Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   typedef typename GraphTypes::edge_descriptor Edge;
   typedef typename boost::property_traits<VState>::value_type::element_type StateCon;
   
public:
   RoadmapGenHalton(
      const ompl::base::StateSpacePtr space,
      const std::string args):
      RoadmapGenSpec(space,"RoadmapGenHalton",args,1),
      dim(0),
      bounds(0),
      num_subgraphs_generated(0),
      vertices_generated(0),
      edges_generated(0)
   {
      // check that we're in a real vector state space
      if (space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapGenHalton only supports rel vector state spaces!");
      dim = space->getDimension();
      if (0 == ompl_multiset::util::get_prime(dim-1))
         throw std::runtime_error("not enough primes hardcoded!");
      bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
      int ret = sscanf(args.c_str(), "n=%u radius=%lf", &n, &radius);
      if (ret != 2)
         throw std::runtime_error("bad args to RoadmapGenHalton!");
      if (args != ompl_multiset::util::sf("n=%u radius=%s",
         n, ompl_multiset::util::double_to_text(radius).c_str()))
      {
         throw std::runtime_error("args not in canonical form!");
      }
   }
   ~RoadmapGenHalton() {}
   
   std::size_t get_num_subgraphs_generated()
   {
      return num_subgraphs_generated;
   }
   
   double root_radius(std::size_t i_subgraph)
   {
      return radius;
   }
   
   void generate(
      Graph & g,
      std::size_t num_subgraphs_desired,
      VState state_map,
      EDistance distance_map,
      VSubgraph vertex_subgraph_map,
      ESubgraph edge_subgraph_map,
      VShadow is_shadow_map)
   {
      if (this->num_subgraphs < num_subgraphs_desired)
         throw std::runtime_error("this roadmap gen doesnt support that many subgraphs !");
      if (num_subgraphs_generated!=0 || num_subgraphs_desired!=1)
         return;
      // ok, generate n nodes!
      while (num_vertices(g) < n)
      {
         Vertex v_new = add_vertex(g);
         
         put(vertex_subgraph_map, v_new, 0);
         put(is_shadow_map, v_new, false);
         
         // allocate a new state for this vertex
         get(state_map, v_new).reset(new StateCon(this->space.get()));
         ompl::base::State * v_state = get(state_map, v_new)->state;
         double * values = v_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
         for (unsigned int ui=0; ui<dim; ui++)
            values[ui] = bounds.low[ui] + (bounds.high[ui] - bounds.low[ui])
               * ompl_multiset::util::halton(
                  ompl_multiset::util::get_prime(ui), vertices_generated);
                  
         // allocate new undirected edges
         for (unsigned int ui=0; ui<num_vertices(g)-1; ui++)
         {
            Vertex v_other = vertex(ui, g);
            double dist = this->space->distance(
               get(state_map, v_new)->state,
               get(state_map, v_other)->state);
            if (radius < dist)
               continue;
            Edge e = add_edge(v_new, v_other, g).first;
            put(distance_map, e, dist);
            put(edge_subgraph_map, e, 0);
            edges_generated++;
         }
         
         vertices_generated++;
      }
      num_subgraphs_generated++;
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
   unsigned int n;
   double radius;
   // progress
   std::size_t num_subgraphs_generated;
   std::size_t vertices_generated;
   std::size_t edges_generated;
};

} // namespace ompl_multiset
