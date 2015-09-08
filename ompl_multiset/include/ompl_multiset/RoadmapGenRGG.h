/* File: RoadmapGenRGG.h
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
   //class StateMap, class SubgraphMap, class IsShadowMap, class DistanceMap
//   >
template <class RoadmapGenSpec>
class RoadmapGenRGG : public RoadmapGenSpec
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
   RoadmapGenRGG(
      const ompl::base::StateSpacePtr space,
      const std::string args):
      RoadmapGenSpec(space,"RoadmapGenRGG",args,1),
      num_subgraphs_generated(0),
      vertices_generated(0),
      edges_generated(0),
      sampler(space->allocStateSampler())
   {
      int ret = sscanf(args.c_str(), "n=%u radius=%lf seed=%u", &n, &radius, &seed);
      if (ret != 3)
         throw std::runtime_error("bad args to RoadmapGenRGG!");
      if (args != ompl_multiset::util::sf("n=%u radius=%s seed=%u",
         n, ompl_multiset::util::double_to_text(radius).c_str(), seed))
      {
         throw std::runtime_error("args not in canonical form!");
      }
      ompl_multiset::SamplerGenMonkeyPatch(sampler) = boost::mt19937(seed);
   }
   ~RoadmapGenRGG() {}
   
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
         this->sampler->sampleUniform(get(state_map, v_new)->state);
         
         // allocate new undirected edges
         for (unsigned int ui=0; ui<num_vertices(g)-1; ui++)
         {
            Vertex v_other = vertex(ui, g);
            double dist = this->space->distance(
               get(state_map, v_new)->state,
               get(state_map, v_other)->state);
            if (this->radius < dist)
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
   // from id
   unsigned int n;
   double radius;
   unsigned int seed;
   // progress
   std::size_t num_subgraphs_generated;
   std::size_t vertices_generated;
   std::size_t edges_generated;
   ompl::base::StateSamplerPtr sampler;
};

} // namespace ompl_multiset
