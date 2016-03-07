/*! \file planner_family.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

/* Semantics: in contrast to most OpenRAVE planners,
 * the family planner is *stateful* between calls to InitPlan().
 * The state can be cleared via a call to ResetFamily().
 * 
 * the family, which is constructed from the msm, must be consistent
 * 
 * todo: planner currently plans with the passed robot's activedofs
 * (not with the robot / activedofs in the param's configspec)
 */
class FamilyPlanner : public OpenRAVE::PlannerBase
{
public:

   // stateful data
   // must be consistent between calls to InitPlan()
   // without a call to ResetFamily()
   // things that change per-plan are:
   // - family indicators and their costs (as current set changes)
   // - the planner's problem definition
   // - internal planner data (of course)
   struct CurrentFamily
   {
      // the family module we're using
      boost::weak_ptr<or_lemur::FamilyModule> mod_family;
      // details about the robot
      // TODO: relax same-robot requirement to family-compatibility
      boost::weak_ptr<OpenRAVE::RobotBase> robot;
      std::vector<int> active_dofs;
      // current family specification (from family module)
      or_lemur::FamilyModule::Family familyspec;
      std::map<or_lemur::FamilyModule::SetPtr,std::string> familyspec_names;
      // the objects used to interface with ompl
      ompl::base::StateSpacePtr ompl_space;
      ompl::base::SpaceInformationPtr ompl_si;
      boost::shared_ptr<ompl_lemur::Family> ompl_family;
      ompl_lemur::FamilyUtilityCheckerPtr ompl_family_checker;
      boost::shared_ptr<ompl_lemur::LEMUR> ompl_lemur;
      
      // put this here for now
      FamilyParametersConstPtr params_last;
   };
   boost::shared_ptr<CurrentFamily> _current_family;
   
   FamilyPlanner(OpenRAVE::EnvironmentBasePtr env);
   ~FamilyPlanner();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr);
   
   bool ResetFamily(std::ostream & sout, std::istream & sin);
   
   bool GetTimes(std::ostream & sout, std::istream & sin) const;
};

} // namespace or_lemur
