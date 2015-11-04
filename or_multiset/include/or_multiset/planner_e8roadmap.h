/* File: planner_e8roadmap.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace or_multiset
{

class E8Roadmap : public OpenRAVE::PlannerBase
{
public:
   
   // <startstate> and <goalstate> can be specified multiple times
   class PlannerParameters : public OpenRAVE::PlannerBase::PlannerParameters
   {
   public:
      std::string roadmap_id;
      unsigned int num_batches_init;
      double coeff_distance;
      double coeff_checkcost;
      double coeff_batch;
      std::string alglog;
      std::string graph;
      PlannerParameters();
   private:
      std::string el_deserializing;
      bool serialize(std::ostream& sout, int options) const;
      OpenRAVE::BaseXMLReader::ProcessElement startElement(
         const std::string & name, const OpenRAVE::AttributesList & atts);
      bool endElement(const std::string & name);
   };
   typedef boost::shared_ptr<PlannerParameters> PlannerParametersPtr;
   typedef boost::shared_ptr<PlannerParameters const> PlannerParametersConstPtr;

   const OpenRAVE::EnvironmentBasePtr env;
   PlannerParametersConstPtr params;
   OpenRAVE::RobotBasePtr robot;
   std::vector<int> robot_adofs;
   ompl::base::StateSpacePtr ompl_space;
   ompl::base::SpaceInformationPtr ompl_si;
   boost::shared_ptr<or_multiset::OrChecker> ompl_checker;
   boost::shared_ptr<ompl_multiset::SimpleEffortModel> sem;
   boost::shared_ptr< ompl_multiset::TagCache<ompl_multiset::E8Roadmap::VIdxTagMap,ompl_multiset::E8Roadmap::EIdxTagsMap> > tag_cache;
   ompl_multiset::E8Roadmap::RoadmapPtr roadmapgen;
   boost::shared_ptr<ompl_multiset::E8Roadmap> ompl_planner;
   ompl::base::ProblemDefinitionPtr ompl_pdef;
   
   E8Roadmap(OpenRAVE::EnvironmentBasePtr env);
   ~E8Roadmap();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr);
   
   bool GetTimes(std::ostream & sout, std::istream & sin) const;
   
};

} // namespace or_multiset
