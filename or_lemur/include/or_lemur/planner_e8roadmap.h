/* File: planner_e8roadmap.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

namespace or_lemur
{

/* 
 * This planner is totally stateless -- a new ompl planner
 * is created on each call to InitPlan()
 */
class E8Roadmap : public OpenRAVE::PlannerBase
{
public:

   const OpenRAVE::EnvironmentBasePtr env;
   E8RoadmapParametersConstPtr params;
   OpenRAVE::RobotBasePtr robot;
   std::vector<int> robot_adofs;
   ompl::base::StateSpacePtr ompl_space;
   ompl::base::SpaceInformationPtr ompl_si;
   boost::shared_ptr<or_lemur::OrChecker> ompl_checker;
   boost::shared_ptr<ompl_lemur::SimpleEffortModel> sem;
   boost::shared_ptr< ompl_lemur::TagCache<ompl_lemur::E8Roadmap::VIdxTagMap,ompl_lemur::E8Roadmap::EIdxTagsMap> > tag_cache;
   //boost::shared_ptr< or_lemur::RoadmapCached<ompl_lemur::E8Roadmap::Roadmap> > roadmapgen;
   boost::shared_ptr<ompl_lemur::E8Roadmap> ompl_planner;
   ompl::base::ProblemDefinitionPtr ompl_pdef;
   
   E8Roadmap(OpenRAVE::EnvironmentBasePtr env);
   ~E8Roadmap();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr);
   
   bool GetTimes(std::ostream & sout, std::istream & sin) const;
};

} // namespace or_lemur
