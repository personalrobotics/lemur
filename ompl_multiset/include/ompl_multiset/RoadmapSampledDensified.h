/* File: RoadmapSampledDensified.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: None
 */

namespace ompl_multiset
{

// for now this is an r-disk prm,
// uniform milestone sampling with given seed,
// with decreasing edge radius based on the rrt* shrinking ball
// uses the space's default sampler
// i think this is a batched version of frazzoli's PRM*?
// gamma_rel > 1 for optimality?
class RoadmapSampledDensified : public Roadmap
{
public:
   // required methods
   RoadmapSampledDensified(
      const ompl::base::StateSpacePtr space,
      unsigned int seed,
      unsigned int batch_n, // number of milestones per subgraph (batch)
      double gamma_rel);
   ~RoadmapSampledDensified();
   
   std::string get_id();
   
   void subgraphs_limit(unsigned int * num_subgraphs);
   void subgraphs_generate(unsigned int num_subgraphs);
   
   void generator_save(std::string & data);
   void generator_load(std::string & data);
   
private:
   std::string id;
   ompl::base::StateSamplerPtr sampler;
   unsigned int batch_n;
   double gamma;
   unsigned int d;
};

} // namespace ompl_multiset
