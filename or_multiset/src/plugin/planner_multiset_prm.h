/* File: planner_multiset_prm.h
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2014, 2015 Carnegie Mellon University
 * License: None
 */

namespace or_multiset
{

class MultiSetPRM : public OpenRAVE::PlannerBase
{
public:

   // <startstate> and <goalstate> can be specified multiple times
   // add lamda
   // add interroot_radius
   class PlannerParameters : public OpenRAVE::PlannerBase::PlannerParameters
   {
   public:
      // extra parameters
      std::vector< std::vector<OpenRAVE::dReal> > startstates;
      std::vector< std::vector<OpenRAVE::dReal> > goalstates;
      int eval_subgraphs;
      double lambda;
      double interroot_radius;
      double timelimit;

      PlannerParameters();
      
   private:
      std::string el_deserializing; // the element we're currently processing
      
      void serialize_startstates(std::ostream & sout) const;
      void serialize_goalstates(std::ostream & sout) const;
      void serialize_eval_subgraphs(std::ostream & sout) const;
      void serialize_lambda(std::ostream & sout) const;
      void serialize_interroot_radius(std::ostream & sout) const;
      void serialize_timelimit(std::ostream & sout) const;
      
      void deserialize_startstate(std::istream & sin);
      void deserialize_goalstate(std::istream & sin);
      void deserialize_eval_subgraphs(std::istream & sin);
      void deserialize_lambda(std::istream & sin);
      void deserialize_interroot_radius(std::istream & sin);
      void deserialize_timelimit(std::istream & sin);

      bool serialize(std::ostream& sout, int options) const;
      OpenRAVE::BaseXMLReader::ProcessElement startElement(
         const std::string & name, const OpenRAVE::AttributesList & atts);
      bool endElement(const std::string & name);
   };
   typedef boost::shared_ptr<PlannerParameters> PlannerParametersPtr;
   typedef boost::shared_ptr<PlannerParameters const> PlannerParametersConstPtr;

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
   PlannerParametersConstPtr params;
   
   // these are all subsets that have been given to the planner
   std::map<std::string, ompl::base::SpaceInformationPtr> subsets;
   std::set< std::pair<std::string, std::vector<std::string> > > intersections;
   
   std::string roadmap_string;
   ompl_multiset::CachePtr cache;
   
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
   bool InitMyPlan(OpenRAVE::RobotBasePtr robot, PlannerParametersConstPtr params);
   
   // this returns the current subset name
   std::string update_planner_current_subsets(OpenRAVE::RobotBasePtr robot);
   
   OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr ptraj);
   
   // SendCommand stuff
   // UseSubsetManager name
   bool UseSubsetManager(std::ostream & sout, std::istream & sin);
   
   // e.g. SetRoadmap class=RoadmapSampledConst seed=419884521 batch_n=1000 radius=2
   // this only stores the arg,
   // may fail on first call to initplan!
   bool SetRoadmap(std::ostream & sout, std::istream & sin);
   
   bool CacheSetLocation(std::ostream & sout, std::istream & sin);
   bool CacheLoad(std::ostream & sout, std::istream & sin);
   bool CacheSave(std::ostream & sout, std::istream & sin);
   
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
