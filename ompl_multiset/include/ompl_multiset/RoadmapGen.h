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
template <class TypeSet>
class RoadmapGen
{
public:
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
      typename TypeSet::Graph & g,
      typename TypeSet::VertexIndexMap vertex_index_map,
      typename TypeSet::EdgeIndexMap edge_index_map,
      typename TypeSet::EdgeVectorMap edge_vector_map,
      std::size_t num_subgraphs_desired,
      typename TypeSet::StateMap state_map,
      typename TypeSet::DistanceMap distance_map,
      typename TypeSet::VertexSubgraphMap vertex_subgraph_map,
      typename TypeSet::EdgeSubgraphMap edge_subgraph_map,
      typename TypeSet::IsShadowMap is_shadow_map) = 0;
   
   virtual void serialize() = 0;
   virtual void deserialize() = 0;
};


//template <class Graph, class VertexIndexMap, class EdgeIndexMap>
//typedef boost::shared_ptr< RoadmapGen<Graph,VertexIndexMap,> > RoadmapGenPtr;

} // namespace ompl_multiset
