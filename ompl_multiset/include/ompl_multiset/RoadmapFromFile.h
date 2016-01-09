/* File: RoadmapFromFile.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_multiset
{

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
      bounds(0)
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
   
   double root_radius(std::size_t i_batch)
   {
      return m_root_radius;
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
      
      std::ifstream fp;
      fp.open(filename.c_str());
      
      boost::dynamic_properties props;
      const ompl::base::StateSpacePtr & myspace = this->space;
      props.property("state",
         ompl_multiset::make_rvstate_map_string_adaptor(
            state_map,
            myspace->as<ompl::base::RealVectorStateSpace>()));
      boost::read_graphml(fp, g, props);
      
      VertexIter vi, vi_end;
      for (boost::tie(vi,vi_end)=vertices(g); vi!=vi_end; ++vi)
      {
         put(vertex_batch_map, *vi, 0);
         put(is_shadow_map, *vi, false);
         nn->add(*vi);
      }
      
      EdgeIter ei, ei_end;
      for (boost::tie(ei,ei_end)=edges(g); ei!=ei_end; ++ei)
      {
         put(edge_batch_map, *ei, 0);
         ompl::base::State * state1 = get(state_map, source(*ei,g));
         ompl::base::State * state2 = get(state_map, target(*ei,g));
         put(distance_map, *ei, this->space->distance(state1, state2));
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
