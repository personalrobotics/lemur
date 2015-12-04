/* File: planner_family.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace or_multiset
{

/* Semantics: in contrast to most OpenRAVE planners,
 * the family planner is *stateful* between calls to InitPlan().
 * The state can be cleared via a call to ClearPlan.
 * 
 * the family, which is constructed from the msm, must be consistent
 */
class FamilyPlanner : public OpenRAVE::PlannerBase
{
public:

   const OpenRAVE::EnvironmentBasePtr env;
   
   // stateful -- must be consistent between calls to InitPlan()
   //boost::weak_ptr<or_multiset::ModuleSubsetManager> w_subset_manager;
   bool plan_initialized;
   boost::weak_ptr<OpenRAVE::RobotBase> w_robot;
   std::vector<int> robot_adofs;
   ompl::base::StateSpacePtr ompl_space;
   ompl_multiset::E8Roadmap::RoadmapPtr roadmapgen;
   boost::shared_ptr<ompl_multiset::Family> family;
   boost::shared_ptr<ompl_multiset::FamilyEffortModel> fem;
   boost::shared_ptr< ompl_multiset::TagCache<ompl_multiset::E8Roadmap::VIdxTagMap,ompl_multiset::E8Roadmap::EIdxTagsMap> > tag_cache;
   boost::shared_ptr<ompl_multiset::E8Roadmap> ompl_planner;
   //boost::shared_ptr<or_multiset::OrChecker> ompl_checker;
   
   // changed arbitrarily on InitPlan()
   E8RoadmapParametersPtr params;
   ompl::base::ProblemDefinitionPtr ompl_pdef;
   
   FamilyPlanner(OpenRAVE::EnvironmentBasePtr env);
   ~FamilyPlanner();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr);
   
   bool ClearPlan(std::ostream & sout, std::istream & sin);
   
   bool GetTimes(std::ostream & sout, std::istream & sin) const;
};

} // namespace or_multiset
