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
template <class TypeSet>
class RoadmapGenRGG : public RoadmapGen<TypeSet>
{
   typedef boost::graph_traits<typename TypeSet::Graph> GraphTypes;
   typedef typename GraphTypes::vertex_descriptor Vertex;
   typedef typename GraphTypes::edge_descriptor Edge;
   
public:
   RoadmapGenRGG(
      const ompl::base::StateSpacePtr space,
      const std::string args):
      RoadmapGen<TypeSet>(space,"RoadmapGenRGG",args,1),
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
   
   void generate(
      typename TypeSet::Graph & g,
      std::size_t num_subgraphs_desired,
      typename TypeSet::StateMap state_map,
      typename TypeSet::DistanceMap distance_map,
      typename TypeSet::VertexSubgraphMap vertex_subgraph_map,
      typename TypeSet::EdgeSubgraphMap edge_subgraph_map,
      typename TypeSet::IsShadowMap is_shadow_map)
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
         get(state_map, v_new).reset(new typename TypeSet::StateContainer(this->space));
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
