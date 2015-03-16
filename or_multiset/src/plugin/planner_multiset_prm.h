namespace or_multiset
{

class MultiSetPRM : public OpenRAVE::PlannerBase
{
public:
   const OpenRAVE::EnvironmentBasePtr penv;

   // in order to work,
   // the planner needs a subset manager module
   // if we aren't explicitly attached to one,
   // initplan will create a private one
   // (note that since the subset manager doesnt used simulationstep
   //  and it doesnt expect to be cloned,
   //  it's ok to use our own private one
   //  without adding it to the environment)
   boost::weak_ptr<or_multiset::ModuleSubsetManager> subset_manager;
   boost::shared_ptr<or_multiset::ModuleSubsetManager> private_subset_manager;

   // these are set by InitPlan and then used by PlanPath
   OpenRAVE::RobotBaseWeakPtr robot;
   std::vector<int> adofs;
   ompl::base::StateSpacePtr ompl_space;
   boost::shared_ptr<ompl_multiset::MultiSetPRM> ompl_planner;
   or_multiset::PlannerParametersConstPtr params;
   
   // these are all subsets that have been given to the planner
   std::map<std::string, ompl::base::SpaceInformationPtr> subsets;
   
   
   // cumulative time in the isvalid function
   // cleared by PlanPath, retrieved by GetTimes
   int n_checks;
   unsigned long long int checktime;
   unsigned long long int totaltime;
   
   //ompl::base::StateSpacePtr space;
   //ompl::base::SpaceInformationPtr spaceinfo;
   //ompl::base::ProblemDefinitionPtr probdef;
   //ompl::base::PlannerPtr planner;
   //boost::shared_ptr<pr_constraint::HolonomicConstraint> holonomic_constraint;

   MultiSetPRM(OpenRAVE::EnvironmentBasePtr penv);
   ~MultiSetPRM();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   bool InitMyPlan(OpenRAVE::RobotBasePtr robot, or_multiset::PlannerParametersConstPtr params);
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr ptraj);
   
   // SendCommand stuff
   // UseSubsetManager name
   bool UseSubsetManager(std::ostream & sout, std::istream & sin);
   
   bool GetTimes(std::ostream & sout, std::istream & sin);
   
   // get currently configured subseg manager
   // (and construct private if missing)
   // may return null pointer if construction failed for some reason
   boost::shared_ptr<or_multiset::ModuleSubsetManager>
      get_subset_manager();
   
   ompl::base::RealVectorBounds ompl_bounds(OpenRAVE::RobotBasePtr robot);
   
   void setup(OpenRAVE::RobotBasePtr robot);
   
   // check of the robot, adofs, bounds, resolutions, etc
   // retrieved during initplan match that of the given robot
   bool setup_isvalid(OpenRAVE::RobotBasePtr robot);
   
   bool ompl_isvalid(
      boost::function<bool (std::vector<OpenRAVE::dReal> &)> indicator,
      const ompl::base::State * s);
};

} /* namespace or_multiset */
