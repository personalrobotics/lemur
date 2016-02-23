/*! \file planner_cctimer.h
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
class CCTimer : public OpenRAVE::PlannerBase
{
public:

   const OpenRAVE::EnvironmentBasePtr env;
   OpenRAVE::RobotBasePtr robot;
   ompl::base::StateSpacePtr ompl_space;
   ompl::base::SpaceInformationPtr ompl_si;
   boost::shared_ptr<or_lemur::OrChecker> ompl_checker;
   
   CCTimer(OpenRAVE::EnvironmentBasePtr env);
   ~CCTimer();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr);
   
   bool GetTimes(std::ostream & sout, std::istream & sin) const;
};

} // namespace or_lemur
