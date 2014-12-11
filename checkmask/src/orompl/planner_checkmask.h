namespace checkmask
{


bool fuzzy_equals(const OpenRAVE::Transform & tx1, const OpenRAVE::Transform & tx2, OpenRAVE::dReal fuzz);





struct TxAjoint
{
   OpenRAVE::Transform tx;
   OpenRAVE::KinBody::JointPtr ajoint;
   bool operator==(const TxAjoint & rhs) const
   {
      if (!fuzzy_equals(tx, rhs.tx, 0.000001)) return false;
      if (ajoint != rhs.ajoint) return false;
      return true;
   }
};

// check between something on the robot (a link or grabbed body)
// and something in the environment
struct InterLinkCheck
{
   // two links, with link1 < link2
   OpenRAVE::KinBody::LinkPtr link1;
   OpenRAVE::KinBody::LinkPtr link2;
   
   // common prefixes removed, first link1 tx identity
   std::vector<TxAjoint> link1_path;
   std::vector<TxAjoint> link2_path;
   
   bool operator==(const InterLinkCheck & rhs) const
   {
      if (link1 != rhs.link1) return false;
      if (link2 != rhs.link2) return false;
      if (link1_path != rhs.link1_path) return false;
      if (link2_path != rhs.link2_path) return false;
      return true;
   }
};

struct Space
{
   std::set<unsigned int> ilcs;
   ompl::base::SpaceInformationPtr ompl_si;
   bool operator==(const Space & rhs) const
   {
      if (ilcs != rhs.ilcs) return false;
      return true;
   }
   bool operator<(const Space & rhs) const
   {
      return (ilcs  < rhs.ilcs);
   }
};

struct Intersection
{
   unsigned int a;
   unsigned int b;
   unsigned int intersection;
};

class OmplCheckMask : public OpenRAVE::PlannerBase
{
public:

   // list of all possibly required inter-link checks
   std::vector<InterLinkCheck> inter_link_checks;

   // spaces used so far (each is a set of inter-link checks)
   // these are the ones we've given to the planner
   // via indexes
   std::vector<Space> spaces;
   
   // some of the spaces are base spaces;
   // we have relations to represent all spaces as intersections between
   // spaces that are eventually in terms of base spaces
   std::set<unsigned int> base_spaces;
   
   std::vector<Intersection> intersections;

   // constants for this planner instance
   OpenRAVE::RobotBasePtr robot;
   std::vector<int> adofs;
   
   unsigned int sidx_current;
   
   ompl::base::StateSpacePtr ompl_space;
   checkmask::GraphPlanner * p;
   
   unsigned long long checktime;
   
   //ompl::base::StateSpacePtr space;
   //ompl::base::SpaceInformationPtr spaceinfo;
   //ompl::base::ProblemDefinitionPtr probdef;
   //ompl::base::PlannerPtr planner;
   //boost::shared_ptr<pr_constraint::HolonomicConstraint> holonomic_constraint;

   OmplCheckMask(OpenRAVE::EnvironmentBasePtr penv);
   ~OmplCheckMask();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   
   //bool InitPlan(OpenRAVE::RobotBasePtr robot, checkmask::PlannerParametersConstPtr params); // custom
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr ptraj);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   // SendCommand stuff
   bool SetOMPLSeed(std::ostream & sout, std::istream & sin);
   bool ListSpaces(std::ostream & sout, std::istream & sin);
   
   // raises exception on inconsistency
   void check_setup(OpenRAVE::RobotBasePtr robot);
   
   // this adds any new required ilcs for this space
   // (but does not add the space itself)
   Space get_current_space();
   
   // this takes the given space and inserts it,
   // also creating any useful base spaces / relations,
   // and also tells them to the planner
   // this returns the corresponding space index
   unsigned int insert_space(Space s);
   
   bool ompl_isvalid(unsigned int sidx, const ompl::base::State * s);
};

} /* namespace pr_constraint_or_ompl */
