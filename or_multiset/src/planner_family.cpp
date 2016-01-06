/* File: planner_family.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <openrave/openrave.h>

#include <boost/chrono.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include <pr_bgl/compose_property_map.hpp>
#include <pr_bgl/edge_indexed_graph.h>
#include <pr_bgl/overlay_manager.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/heap_indexed.h>

#include <ompl_multiset/aborting_space_information.h>
#include <ompl_multiset/util.h>
#include <ompl_multiset/rvstate_map_string_adaptor.h>
#include <ompl_multiset/EffortModel.h>
#include <ompl_multiset/Family.h>
#include <ompl_multiset/FamilyEffortModel.h>
#include <ompl_multiset/FnString.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapAAGrid.h>
#include <ompl_multiset/RoadmapFromFile.h>
#include <ompl_multiset/RoadmapHalton.h>
#include <ompl_multiset/RoadmapHaltonDens.h>
#include <ompl_multiset/RoadmapHaltonOffDens.h>
#include <ompl_multiset/RoadmapRGG.h>
#include <ompl_multiset/RoadmapRGGDens.h>
#include <ompl_multiset/RoadmapRGGDensConst.h>
#include <ompl_multiset/RoadmapID.h>
#include <ompl_multiset/BisectPerm.h>
#include <ompl_multiset/E8Roadmap.h>

#include <or_multiset/inter_link_checks.h>
#include <or_multiset/module_subset_manager.h>
#include <or_multiset/or_checker.h>
#include <or_multiset/params_e8roadmap.h>
#include <or_multiset/planner_family.h>


namespace {

ompl::base::RealVectorBounds ompl_bounds(OpenRAVE::RobotBasePtr robot)
{
   ompl::base::RealVectorBounds bounds(robot->GetActiveDOF());
   std::vector<OpenRAVE::dReal> lowers;
   std::vector<OpenRAVE::dReal> uppers;
   robot->GetActiveDOFLimits(lowers, uppers);
   for (int i=0; i<robot->GetActiveDOF(); i++)
   {
      bounds.setLow(i, lowers[i]);
      bounds.setHigh(i, uppers[i]);
   }
   return bounds;
}

double ompl_resolution(OpenRAVE::RobotBasePtr robot)
{
   std::vector<OpenRAVE::dReal> dof_resolutions;
   robot->GetActiveDOFResolutions(dof_resolutions);
   double resolution = HUGE_VAL;
   for (unsigned int i=0; i<dof_resolutions.size(); i++)
      resolution = dof_resolutions[i] < resolution ? dof_resolutions[i] : resolution;
   return resolution;
}

void ompl_set_roots(ompl::base::ProblemDefinitionPtr ompl_pdef,
   OpenRAVE::PlannerBase::PlannerParametersConstPtr params)
{
   ompl::base::SpaceInformationPtr space_si = ompl_pdef->getSpaceInformation();
   ompl::base::StateSpacePtr space = space_si->getStateSpace();
   unsigned int dim = space->getDimension();
   
   // add start states
   ompl_pdef->clearStartStates();
   if (params->vinitialconfig.size() % dim != 0)
      throw OpenRAVE::openrave_exception("vector of initial states is not the right size!");
   unsigned int num_starts = params->vinitialconfig.size() / dim;
   for (unsigned int istart=0; istart<num_starts; istart++)
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(space);
      for (unsigned int j=0; j<dim; j++)
         s_start->values[j] = params->vinitialconfig[istart*dim + j];
      ompl_pdef->addStartState(s_start);
   }
   
   // add goal states
   ompl::base::GoalStates * gs = new ompl::base::GoalStates(space_si);
   gs->clear();
   if (params->vgoalconfig.size() % dim != 0)
      throw OpenRAVE::openrave_exception("vector of goal states is not the right size!");
   unsigned int num_goals = params->vgoalconfig.size() / dim;
   for (unsigned int igoal=0; igoal<num_goals; igoal++)
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(space);
      for (unsigned int j=0; j<dim; j++)
         s_goal->values[j] = params->vgoalconfig[igoal*dim + j];
      gs->addState(s_goal);
   }
   ompl_pdef->setGoal(ompl::base::GoalPtr(gs));
}

// find the subset manager module in the env with this name
boost::shared_ptr<or_multiset::ModuleSubsetManager>
find_subset_manager(const OpenRAVE::EnvironmentBasePtr & env, const std::string & name)
{
   // get all modules
   std::list<OpenRAVE::ModuleBasePtr> modules;
   env->GetModules(modules);
   
   // find matches
   std::vector< boost::shared_ptr<or_multiset::ModuleSubsetManager> > matches;
   for (std::list<OpenRAVE::ModuleBasePtr>::iterator it=modules.begin(); it!=modules.end(); it++)
   {
      boost::shared_ptr<or_multiset::ModuleSubsetManager> mod
         = boost::dynamic_pointer_cast<or_multiset::ModuleSubsetManager>(*it);
      if (!mod)
         continue;
      if (mod->name != name)
         continue;
      matches.push_back(mod);
   }
   
   if (matches.size() == 0)
   {
      RAVELOG_ERROR("No subset manager module has name \"%s\"!\n", name.c_str());
      return boost::shared_ptr<or_multiset::ModuleSubsetManager>();
   }
   
   if (matches.size() != 1)
   {
      RAVELOG_ERROR("Not exactly one subset manager module has name \"%s\"! (THIS SHOULD NEVER HAPPEN)\n", name.c_str());
      return boost::shared_ptr<or_multiset::ModuleSubsetManager>();
   }
   
   return matches[0];
}

class OrIndicatorChecker: public ompl::base::StateValidityChecker
{
public:
   boost::function<bool (std::vector<OpenRAVE::dReal> &)> indicator;
   const size_t dim;
   mutable size_t num_checks;
   mutable boost::chrono::high_resolution_clock::duration dur_checks;
   OrIndicatorChecker(
      const ompl::base::SpaceInformationPtr & si,
      boost::function<bool (std::vector<OpenRAVE::dReal> &)> indicator,
      const size_t dim):
      ompl::base::StateValidityChecker(si),
      indicator(indicator), dim(dim), num_checks(0), dur_checks()
   {
   }
   bool isValid(const ompl::base::State * state) const
   {
      boost::chrono::high_resolution_clock::time_point time_begin
         = boost::chrono::high_resolution_clock::now();
      num_checks++;
      double * q = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      std::vector<OpenRAVE::dReal> adofvals(q, q+dim);
      bool is_valid = indicator(adofvals);
      dur_checks += boost::chrono::high_resolution_clock::now() - time_begin;
      return is_valid;
      
   }
};

} // anonymous namespace


or_multiset::FamilyPlanner::FamilyPlanner(OpenRAVE::EnvironmentBasePtr env):
   OpenRAVE::PlannerBase(env), env(env), plan_initialized(false)
{
   __description = "Family Planner (based on E8)";
   RegisterCommand("ClearPlan",
      boost::bind(&or_multiset::FamilyPlanner::ClearPlan,this,_1,_2),
      "get timing information from last plan");
   RegisterCommand("GetTimes",
      boost::bind(&or_multiset::FamilyPlanner::GetTimes,this,_1,_2),
      "get timing information from last plan");
}

or_multiset::FamilyPlanner::~FamilyPlanner()
{
}

bool
or_multiset::FamilyPlanner::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & inparams_ser)
{
   or_multiset::E8RoadmapParametersPtr inparams(new or_multiset::E8RoadmapParameters());
   inparams_ser >> *inparams;
   inparams->Validate();
   return this->InitPlan(robot, inparams);
}

bool
or_multiset::FamilyPlanner::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr inparams_base)
{
   if (!robot || !inparams_base)
   {
      RAVELOG_ERROR("robot/params objects must be passed!\n");
      return false;
   }
   E8RoadmapParametersConstPtr inparams = boost::dynamic_pointer_cast<or_multiset::E8RoadmapParameters const>(inparams_base);
   if (!inparams)
   {
      std::stringstream inparams_ser;
      inparams_ser << *inparams_base;
      return InitPlan(robot, inparams_ser);
   }
   
   // retrieve subset manager
   boost::shared_ptr<or_multiset::ModuleSubsetManager> subset_manager = find_subset_manager(env, "my_ssm");
   if (!subset_manager)
   {
      RAVELOG_ERROR("subset_manager required!\n");
      return false;
   }
   
   if (!plan_initialized)
   {
      // do setup (at most once)
      params.reset(new or_multiset::E8RoadmapParameters());
      params->copy(inparams);
      
      // robot stuff
      w_robot = robot;
      robot_adofs = robot->GetActiveDOFIndices();
      
      // construct ompl space
      ompl_space.reset(new ompl::base::RealVectorStateSpace(robot_adofs.size()));
      ompl_space->as<ompl::base::RealVectorStateSpace>()->setBounds(ompl_bounds(robot));
      ompl_space->setLongestValidSegmentFraction(ompl_resolution(robot) / ompl_space->getMaximumExtent());
      ompl_space->setup();
      
      // construct roadmap
      if (!params->has_roadmap_id)
      {
         RAVELOG_ERROR("roadmap_id parameter was not passed!\n");
         return false;
      }
      try
      {
         roadmapgen.reset(ompl_multiset::make_roadmap_gen<ompl_multiset::E8Roadmap::Roadmap>(ompl_space, params->roadmap_id));
      }
      catch (const std::runtime_error & ex)
      {
         RAVELOG_ERROR("failure to create roadmap!\n");
         return false;
      }
      
      // get current family report from subset manager
      or_multiset::SubsetReport subset_report;
      subset_manager->get_current_report(robot, subset_report);
      
      // construct family based on ompl-agnostic subset report
      family.reset(new ompl_multiset::Family());
      
      // convert each subset
      for (std::vector<or_multiset::SubsetReport::Subset>::iterator
         it=subset_report.subsets.begin(); it!=subset_report.subsets.end(); it++)
      {
         // create new space_info
         ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(ompl_space));
         // bind to the subset manarger's indicator function
         si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
            new OrIndicatorChecker(si, it->indicator, robot_adofs.size())
         ));
         // add this subset
         family->subsets.insert(std::make_pair(
            it->name, ompl_multiset::Family::Subset(si, it->cost)));
      }
      
      // add inclusions
      for (std::vector<or_multiset::SubsetReport::Inclusion>::iterator
         it=subset_report.inclusions.begin(); it!=subset_report.inclusions.end(); it++)
      {
         family->inclusions.insert(ompl_multiset::Family::Inclusion(it->subset, it->superset));
      }
      
      // add intersections
      for (std::vector<or_multiset::SubsetReport::Intersection>::iterator
         it=subset_report.intersections.begin(); it!=subset_report.intersections.end(); it++)
      {
         std::set<std::string> supersets(it->supersets.begin(), it->supersets.end());
         family->intersections.insert(ompl_multiset::Family::Intersection(it->subset,supersets));
      }
      
      // create the family effort model
      // (note -- we haven't yet set the target set!)
      fem.reset(new ompl_multiset::FamilyEffortModel(*family));
      //fem->set_target(family->subsets.find("targ")->second.si);
      
      // punt on tag cache for now
      tag_cache.reset(new ompl_multiset::DummyTagCache<ompl_multiset::E8Roadmap::VIdxTagMap,ompl_multiset::E8Roadmap::EIdxTagsMap>());
      
      ompl_planner.reset(new ompl_multiset::E8Roadmap(ompl_space, *fem, *tag_cache, roadmapgen));
      
      plan_initialized = true;
   }
   
   // check consistency
   // (note these checks should pass trivially if this is the first InitPlan)
   if (robot != w_robot.lock())
   {
      RAVELOG_ERROR("stateful planner supports only one robot!\n");
      return false;
   }
   if (robot_adofs != robot->GetActiveDOFIndices())
   {
      RAVELOG_ERROR("stateful planner supports only one robot adofs!\n");
      return false;
   }
   if (inparams->has_roadmap_id && inparams->roadmap_id != params->roadmap_id)
   {
      RAVELOG_ERROR("stateful planner supports only one roadmap_id!\n");
      return false;
   }
   
   // determine current set
   // this can throw!
   {
      or_multiset::SubsetReport subset_report;
      subset_manager->get_current_report(robot, subset_report);
      std::string current_set = subset_report.current_subset;
      RAVELOG_INFO("planning in target set \"%s\" ...\n", current_set.c_str());
      std::map<std::string, ompl_multiset::Family::Subset>::iterator fam_set = family->subsets.find(current_set);
      if (fam_set == family->subsets.end())
      {
         RAVELOG_ERROR("current set named \"%s\" not found in family!\n", current_set.c_str());
         return false;
      }
      fem->set_target(fam_set->second.si);
   }
   
   // set params
   params.reset(new or_multiset::E8RoadmapParameters());
   params->copy(inparams);
   
   // planner params
   if (params->has_coeff_distance)
      ompl_planner->setCoeffDistance(params->coeff_distance);
   if (params->has_coeff_checkcost)
      ompl_planner->setCoeffCheckcost(params->coeff_checkcost);
   if (params->has_coeff_batch)
      ompl_planner->setCoeffBatch(params->coeff_batch);
   if (params->has_do_timing)
      ompl_planner->setDoTiming(params->do_timing);
   if (params->has_persist_roots)
      ompl_planner->setPersistRoots(params->persist_roots);
   if (params->has_num_batches_init)
      ompl_planner->setNumBatchesInit(params->num_batches_init);
   if (params->has_max_batches)
      ompl_planner->setMaxBatches(params->max_batches);
   if (params->has_search_type)
      ompl_planner->setSearchType(params->search_type);
   if (params->has_eval_type)
      ompl_planner->setEvalType(params->eval_type);
   
   // problem definition
   ompl_pdef.reset(new ompl::base::ProblemDefinition(
      ompl_multiset::get_aborting_space_information(ompl_space)));
   ompl_set_roots(ompl_pdef, params);
   ompl_planner->setProblemDefinition(ompl_pdef);
   
   return true;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr
or_multiset::FamilyPlanner::GetParameters() const
{
   return params;
}

OpenRAVE::PlannerStatus
or_multiset::FamilyPlanner::PlanPath(OpenRAVE::TrajectoryBasePtr traj)
{
   OpenRAVE::RobotBasePtr robot(w_robot.lock());
   if (!robot)
      throw OpenRAVE::openrave_exception("was InitPlan called, or was robot removed from env?");
   
   printf("planning ...\n");
   
#if 0
   ompl_checker->num_checks = 0;
   ompl_checker->dur_checks = boost::chrono::high_resolution_clock::duration();
#endif

   std::ofstream fp_alglog;
   if (params->alglog == "-")
   {
      ompl_planner->as<ompl_multiset::E8Roadmap>()->os_alglog = &std::cout;
   }
   else if (params->alglog != "")
   {
      if (params->has_do_alglog_append && params->do_alglog_append)
         fp_alglog.open(params->alglog.c_str(), std::ios_base::app);
      else
         fp_alglog.open(params->alglog.c_str(), std::ios_base::out);
      ompl_planner->as<ompl_multiset::E8Roadmap>()->os_alglog = &fp_alglog;
   }
   
   ompl::base::PlannerStatus ompl_status;
   ompl::base::PlannerTerminationCondition ptc(ompl::base::plannerNonTerminatingCondition());
   if (params->has_time_limit)
      ptc = ompl::base::timedPlannerTerminationCondition(params->time_limit);
   ompl_status = ompl_planner->solve(ptc);
   
#if 0
   printf("planner returned: %s\n", ompl_status.asString().c_str());
   printf("planner performed %lu checks!\n", ompl_checker->num_checks);
#endif

   ompl_planner->as<ompl_multiset::E8Roadmap>()->os_alglog = 0;
   fp_alglog.close();
   
   if (params->graph == "-")
   {
      ompl_planner->as<ompl_multiset::E8Roadmap>()->dump_graph(std::cout);
   }
   else if (params->graph != "")
   {
      std::ofstream fp_graph;
      fp_graph.open(params->graph.c_str());
      ompl_planner->as<ompl_multiset::E8Roadmap>()->dump_graph(fp_graph);
      fp_graph.close();
   }
   
   if (ompl_status != ompl::base::PlannerStatus::EXACT_SOLUTION) return OpenRAVE::PS_Failed;
   
   // convert result
   // (if the planner exited with an exact empty solution, then it's done!)
   ompl::base::PathPtr path = this->ompl_planner->getProblemDefinition()->getSolutionPath();
   if (!path)
      return OpenRAVE::PS_Failed;
   
   ompl::geometric::PathGeometric * gpath = dynamic_cast<ompl::geometric::PathGeometric*>(path.get());
   if (!gpath)
      throw OpenRAVE::openrave_exception("ompl path is not geometric for some reason!");
   traj->Init(robot->GetActiveConfigurationSpecification());
   for (unsigned int i=0; i<gpath->getStateCount(); i++)
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(ompl_space, gpath->getState(i));
      traj->Insert(i, std::vector<OpenRAVE::dReal>(&s[0], &s[0]+robot->GetActiveDOF()));
   }
   
   return OpenRAVE::PS_HasSolution;
}

bool or_multiset::FamilyPlanner::ClearPlan(std::ostream & sout, std::istream & sin)
{
   w_robot.reset();
   robot_adofs.clear();
   ompl_space.reset();
   roadmapgen.reset();
   family.reset();;
   fem.reset();
   tag_cache.reset();
   ompl_planner.reset();
   params.reset();
   ompl_pdef.reset();
   plan_initialized = false;
   return true;
}

bool or_multiset::FamilyPlanner::GetTimes(std::ostream & sout, std::istream & sin) const
{
   //sout << "checktime " << boost::chrono::duration<double>(ompl_checker->dur_checks).count();
   //sout << " totaltime " << 0.0;
   if (family)
   {
      for (std::map<std::string, ompl_multiset::Family::Subset>::iterator
         it=family->subsets.begin(); it!=family->subsets.end(); it++)
      {
         sout << " n_checks_" << it->first;
         sout << " " << ((OrIndicatorChecker*)it->second.si->getStateValidityChecker().get())->num_checks;
      }
   }
   sout << " e8_dur_total " <<  ompl_planner->as<ompl_multiset::E8Roadmap>()->getDurTotal();
   sout << " e8_dur_search " <<  ompl_planner->as<ompl_multiset::E8Roadmap>()->getDurSearch();
   sout << " e8_dur_eval " <<  ompl_planner->as<ompl_multiset::E8Roadmap>()->getDurEval();
   return true;
}
