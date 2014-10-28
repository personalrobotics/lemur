namespace checkmask
{

class OmplCheckMask : public OpenRAVE::PlannerBase
{
public:
   //OpenRAVE::RobotBasePtr robot;
   //ompl::base::StateSpacePtr space;
   //ompl::base::SpaceInformationPtr spaceinfo;
   //ompl::base::ProblemDefinitionPtr probdef;
   //ompl::base::PlannerPtr planner;
   //boost::shared_ptr<pr_constraint::HolonomicConstraint> holonomic_constraint;

   OmplCheckMask(OpenRAVE::EnvironmentBasePtr penv);
   ~OmplCheckMask();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr ptraj);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   //bool ompl_isstatevalid(const ompl::base::State * state);
};

} /* namespace pr_constraint_or_ompl */
