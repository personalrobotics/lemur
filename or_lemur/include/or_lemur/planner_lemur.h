/*! \file planner_lemur.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

/* 
 * This planner is totally stateless -- a new ompl planner
 * is created on each call to InitPlan()
 */
class LEMUR : public OpenRAVE::PlannerBase
{
public:

   const OpenRAVE::EnvironmentBasePtr env;
   LEMURParametersConstPtr params;
   OpenRAVE::RobotBasePtr robot;
   std::vector<int> robot_adofs;
   ompl::base::StateSpacePtr ompl_space;
   ompl::base::SpaceInformationPtr ompl_si;
   boost::shared_ptr<or_lemur::OrChecker> ompl_checker;
   //boost::shared_ptr<ompl_lemur::SimpleEffortModel> sem;
   ompl_lemur::BinaryUtilityCheckerPtr ompl_binary_checker;
   boost::shared_ptr< ompl_lemur::TagCache<ompl_lemur::LEMUR::VIdxTagMap,ompl_lemur::LEMUR::EIdxTagsMap> > tag_cache;
   //boost::shared_ptr< or_lemur::RoadmapCached<ompl_lemur::LEMUR::Roadmap> > roadmapgen;
   boost::shared_ptr<ompl_lemur::LEMUR> ompl_planner;
   ompl::base::ProblemDefinitionPtr ompl_pdef;
   
   LEMUR(OpenRAVE::EnvironmentBasePtr env);
   ~LEMUR();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   
   /*! \brief Initialize a LEMUR plan.
    */
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr);
   
   bool GetTimes(std::ostream & sout, std::istream & sin) const;
};

} // namespace or_lemur
