/*! \file planner_family.h
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

namespace or_lemur
{

/*! \brief OpenRAVE implementation of the Family planner, which
 *         plans over the family defined by the family module.
 * 
 * Semantics: in contrast to most OpenRAVE planners,
 * the family planner is *stateful* between calls to InitPlan().
 * The (family) state can be cleared via a call to ResetFamily().
 * 
 * The family (i.e. the robot kinematics hash and its activedofs)
 * is constructed from the family module and must be consistent.
 * 
 * Roadmap params are only meaningful on the first call to InitPlan(),
 * and cannot be changed between calls.
 * 
 * Cached set params -- the sets which have associated cache files
 * if these sets don't already exist in the family, they will be added
 * (transiently, like $live) -- to name them, name them in the family!
 * 
 * \todo planner currently plans with the passed robot's activedofs
 * (not with the robot / activedofs in the param's configspec)
 * 
 * for now, the family and setcaches must be consistent between calls
 * \todo allow dynamic family and setcaches
 */
class FamilyPlanner : public OpenRAVE::PlannerBase
{
public:

   struct SetCache
   {
      std::string name;
      std::string filename;
      std::string set_header; // from start of file, opaque (passed to module)
      std::string roadmap_header; // read from file
      FamilyModule::SetPtr set;
   };

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
      // caches (from params), key is filename
      std::map<std::string, SetCache> setcaches;
      // current set
      FamilyModule::SetPtr set_current;
      // current family specification (from family module)
      or_lemur::FamilyModule::Family familyspec;
      std::map<or_lemur::FamilyModule::SetPtr,std::string> familyspec_names;
      // the objects used to interface with ompl
      ompl::base::SpaceInformationPtr ompl_si;
      boost::shared_ptr<ompl_lemur::Family> ompl_family;
      ompl_lemur::FamilyUtilityCheckerPtr ompl_family_checker;
      boost::shared_ptr<
         ompl_lemur::FamilyTagCache<ompl_lemur::LEMUR::VIdxTagMap,ompl_lemur::LEMUR::EIdxTagsMap>
         > ompl_tag_cache;
      boost::shared_ptr<ompl_lemur::LEMUR> ompl_lemur;
      // roadmap header (known once planner initializes)
      std::string roadmap_header;
      
      // put this here for now
      FamilyParametersConstPtr params_last;
   };
   boost::shared_ptr<CurrentFamily> _current_family;
   
   bool _initialized;
   
   FamilyPlanner(OpenRAVE::EnvironmentBasePtr env);
   ~FamilyPlanner();
   
   bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & isParameters);
   bool InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params);
   OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const;
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr);
   
   bool CmdSaveSetCaches(std::ostream & sout, std::istream & sin);
   
   bool CmdResetFamily(std::ostream & sout, std::istream & sin);
   
   bool CmdGetTimes(std::ostream & sout, std::istream & sin) const;
};

} // namespace or_lemur
