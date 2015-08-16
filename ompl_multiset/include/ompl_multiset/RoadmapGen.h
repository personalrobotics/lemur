/* File: RoadmapGen.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

// continuous space
// generates a possibly infinite roadmap given an ompl space
// note, generate() should tolerate the graph having unused but
// added vertices (e.g. from old/unapplied roots)
//template <class Graph, class VertexIndexMap, class EdgeIndexMap
   //,class StateMap, class SubgraphMap, class IsShadowMap, class DistanceMap
//   >
template <class Graph, class VState, class EDistance, class VSubgraph, class ESubgraph, class VShadow>
class RoadmapGen
{
public:
   typedef Graph BaseGraph;
   typedef VState BaseVState;
   typedef EDistance BaseEDistance;
   typedef VSubgraph BaseVSubgraph;
   typedef ESubgraph BaseESubgraph;
   typedef VShadow BaseVShadow;

   const ompl::base::StateSpacePtr space;
   const std::string type;
   const std::string args;
   const std::size_t num_subgraphs; // 0 means inf

   RoadmapGen(
      const ompl::base::StateSpacePtr space,
      const std::string type,
      const std::string args,
      int num_subgraphs):
      space(space), type(type), args(args), num_subgraphs(num_subgraphs)
   {
   }
   virtual ~RoadmapGen() {}
   
   virtual std::size_t get_num_subgraphs_generated() = 0;
   
   // sets all of these maps
   virtual void generate(
      Graph & g,
      std::size_t num_subgraphs_desired,
      VState state_map,
      EDistance distance_map,
      VSubgraph vertex_subgraph_map,
      ESubgraph edge_subgraph_map,
      VShadow is_shadow_map) = 0;
   
   virtual void serialize() = 0;
   virtual void deserialize() = 0;
};

} // namespace ompl_multiset
