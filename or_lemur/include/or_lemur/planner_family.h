/*! \file planner_family.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
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
   //boost::weak_ptr<or_lemur::ModuleSubsetManager> w_subset_manager;
   bool plan_initialized;
   boost::weak_ptr<OpenRAVE::RobotBase> w_robot;
   std::vector<int> robot_adofs;
   ompl::base::StateSpacePtr ompl_space;
   boost::shared_ptr<ompl_lemur::Family> family;
   boost::shared_ptr<ompl_lemur::FamilyEffortModel> fem;
   boost::shared_ptr< ompl_lemur::TagCache<ompl_lemur::LEMUR::VIdxTagMap,ompl_lemur::LEMUR::EIdxTagsMap> > tag_cache;
   boost::shared_ptr<ompl_lemur::LEMUR> ompl_planner;
   //boost::shared_ptr<or_lemur::OrChecker> ompl_checker;
   
   // changed arbitrarily on InitPlan()
   LEMURParametersPtr params;
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

} // namespace or_lemur
