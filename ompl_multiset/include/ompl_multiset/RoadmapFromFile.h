/* File: RoadmapFromFile.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

// turns values from doubles to strings
template <class PropMap, class StateCon>
class RoadmapFromFilePutStateMap
{
public:
   typedef boost::writable_property_map_tag category;
   typedef typename boost::property_traits<PropMap>::key_type key_type;
   typedef std::string value_type;
   typedef std::string reference;
   const PropMap prop_map;
   ompl::base::StateSpace * space;
   const unsigned int dim;
   RoadmapFromFilePutStateMap(PropMap prop_map, ompl::base::StateSpace * space, unsigned int dim):
      prop_map(prop_map), space(space), dim(dim)
   {
   }
};

template <class PropMap, class StateCon>
inline std::string
get(const RoadmapFromFilePutStateMap<PropMap,StateCon> & map,
   const typename RoadmapFromFilePutStateMap<PropMap,StateCon>::key_type & k)
{
   printf("get on RoadmapFromFilePutStateMap not yet implemented!\n");
   abort();
}

template <class PropMap, class StateCon>
inline void
put(const RoadmapFromFilePutStateMap<PropMap,StateCon> & map,
   const typename RoadmapFromFilePutStateMap<PropMap,StateCon>::key_type & k,
   const std::string repr)
{
   get(map.prop_map, k).reset(new StateCon(map.space));
   ompl::base::State * v_state = get(map.prop_map, k)->state;
   double * values = v_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   std::stringstream ss(repr);
   for (unsigned int ui=0; ui<map.dim; ui++)
      ss >> values[ui];
}

// for now this is an r-disk prm,
// uniform milestone sampling with given seed,
// uses the space's default sampler
//template <class Graph, class VertexIndexMap, class EdgeIndexMap//,
   //class StateMap, class BatchMap, class IsShadowMap, class DistanceMap
//   >
template <class RoadmapSpec>
class RoadmapFromFile : public RoadmapSpec
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
   typedef typename GraphTypes::vertex_iterator VertexIter;
   typedef typename GraphTypes::edge_descriptor Edge;
   typedef typename GraphTypes::edge_iterator EdgeIter;
   typedef typename boost::property_traits<VState>::value_type::element_type StateCon;
   
public:

   // parameters
   const std::string filename;
   const std::string filesha1;
   const double m_root_radius;

   RoadmapFromFile(
      const ompl::base::StateSpacePtr space,
      std::string filename, double root_radius):
      RoadmapSpec(space,1),
      filename(filename),
      filesha1(ompl_multiset::util::file_sha1(filename)),
      m_root_radius(root_radius),
      bounds(0),
      num_batches_generated(0)
   {
      // check that we're in a real vector state space
      if (space->getType() != ompl::base::STATE_SPACE_REAL_VECTOR)
         throw std::runtime_error("RoadmapHalton only supports rel vector state spaces!");
      dim = space->getDimension();
      if (0 == ompl_multiset::util::get_prime(dim-1))
         throw std::runtime_error("not enough primes hardcoded!");
      bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
   }
   ~RoadmapFromFile() {}
   
   std::size_t get_num_batches_generated()
   {
      return num_batches_generated;
   }
   
   double root_radius(std::size_t i_batch)
   {
      return m_root_radius;
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
      
      std::ifstream fp;
      fp.open(filename.c_str());
      
      boost::dynamic_properties props;
      props.property("state",
         RoadmapFromFilePutStateMap<VState,StateCon>(state_map, this->space.get(), dim));
      boost::read_graphml(fp, g, props);
      
      VertexIter vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      {
         put(vertex_batch_map, *vi, 0);
         put(is_shadow_map, *vi, false);
      }
      
      EdgeIter ei, ei_end;
      for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ++ei)
      {
         put(edge_batch_map, *ei, 0);
         ompl::base::State * state1 = get(state_map, source(*ei,g))->state;
         ompl::base::State * state2 = get(state_map, target(*ei,g))->state;
         put(distance_map, *ei, this->space->distance(state1, state2));
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
   // progress
   std::size_t num_batches_generated;
};

} // namespace ompl_multiset
