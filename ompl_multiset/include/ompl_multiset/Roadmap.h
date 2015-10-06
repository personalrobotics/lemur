/* File: Roadmap.h
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
   //,class StateMap, class BatchMap, class IsShadowMap, class DistanceMap
//   >
template <class Graph, class VState, class EDistance, class VBatch, class EBatch, class VShadow>
class Roadmap
{
public:
   typedef Graph BaseGraph;
   typedef VState BaseVState;
   typedef EDistance BaseEDistance;
   typedef VBatch BaseVBatch;
   typedef EBatch BaseEBatch;
   typedef VShadow BaseVShadow;

   const ompl::base::StateSpacePtr space;
   const std::string type;
   const std::string args;
   const std::size_t max_batches; // 0 means inf
   
   // TODO: make this per-type, and move string parsing into RoadmapID
   Roadmap(
      const ompl::base::StateSpacePtr space,
      const std::string type,
      const std::string args,
      int max_batches):
      space(space), type(type), args(args), max_batches(max_batches)
   {
   }
   virtual ~Roadmap() {}
   
   virtual std::size_t get_num_batches_generated() = 0;
   
   virtual double root_radius(std::size_t i_batch) = 0;
   
   // sets all of these maps
   // generates one additional batch
   virtual void generate(
      Graph & g,
      VState state_map,
      EDistance distance_map,
      VBatch vertex_batch_map,
      EBatch edge_batch_map,
      VShadow is_shadow_map) = 0;
   
   virtual void serialize() = 0;
   virtual void deserialize() = 0;
};

} // namespace ompl_multiset
