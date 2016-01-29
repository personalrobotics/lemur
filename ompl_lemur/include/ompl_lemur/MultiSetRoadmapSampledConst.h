/* File: RoadmapSampledConst.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace ompl_lemur
{

// for now this is an r-disk prm,
// uniform milestone sampling with given seed,
// uses the space's default sampler
class MultiSetRoadmapSampledConst : public MultiSetRoadmap
{
public:
   // required methods
   MultiSetRoadmapSampledConst(
      const ompl::base::StateSpacePtr space,
      unsigned int seed,
      unsigned int batch_n, // number of milestones per subgraph (batch)
      double radius);
   ~MultiSetRoadmapSampledConst();
   
   std::string get_id();
   
   void subgraphs_limit(unsigned int * num_subgraphs);
   void subgraphs_generate(unsigned int num_subgraphs);
   
   void generator_save(std::string & data);
   void generator_load(std::string & data);
   
private:
   std::string id;
   ompl::base::StateSamplerPtr sampler;
   unsigned int batch_n;
   double radius;
};

} // namespace ompl_lemur
