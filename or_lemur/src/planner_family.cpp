/* File: planner_family.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <openrave/openrave.h>
#include <openrave/utils.h>

#include <boost/chrono.hpp>
#include <boost/filesystem.hpp>
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

#include <ompl_lemur/util.h>
#include <ompl_lemur/rvstate_map_string_adaptor.h>
#include <ompl_lemur/TagCache.h>
#include <ompl_lemur/Family.h>
#include <ompl_lemur/UtilityChecker.h>
#include <ompl_lemur/FamilyUtilityChecker.h>
#include <ompl_lemur/FnString.h>
#include <ompl_lemur/SpaceID.h>
#include <ompl_lemur/SamplerGenMonkeyPatch.h>
#include <ompl_lemur/NearestNeighborsLinearBGL.h>
#include <ompl_lemur/Roadmap.h>
#include <ompl_lemur/RoadmapAAGrid.h>
#include <ompl_lemur/RoadmapFromFile.h>
#include <ompl_lemur/RoadmapHalton.h>
#include <ompl_lemur/RoadmapHaltonDens.h>
#include <ompl_lemur/RoadmapHaltonOffDens.h>
#include <ompl_lemur/RoadmapRGG.h>
#include <ompl_lemur/RoadmapRGGDens.h>
#include <ompl_lemur/RoadmapRGGDensConst.h>
#include <ompl_lemur/BisectPerm.h>
#include <ompl_lemur/LEMUR.h>

#include <or_lemur/RoadmapCached.h>
#include <or_lemur/inter_link_checks.h>
#include <or_lemur/module_subset_manager.h>
#include <or_lemur/or_checker.h>
#include <or_lemur/params_lemur.h>
#include <or_lemur/params_family.h>
#include <or_lemur/module_family.h>
#include <or_lemur/planner_family.h>


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

class OrIndicatorChecker: public ompl::base::StateValidityChecker
{
public:
   boost::function<bool (std::vector<OpenRAVE::dReal> &)> indicator;
   const size_t dim;
   mutable size_t num_checks;
   mutable boost::chrono::high_resolution_clock::duration dur_checks;
   OrIndicatorChecker(
      const ompl::base::SpaceInformationPtr & si,
      boost::function<bool (std::vector<OpenRAVE::dReal> &)> indicator):
      ompl::base::StateValidityChecker(si),
      indicator(indicator), dim(si->getStateDimension()),
      num_checks(0), dur_checks()
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


or_lemur::FamilyPlanner::FamilyPlanner(OpenRAVE::EnvironmentBasePtr env):
   OpenRAVE::PlannerBase(env)
{
   __description = "Family Planner (based on LEMUR)";
   RegisterCommand("Reset",
      boost::bind(&or_lemur::FamilyPlanner::ResetFamily,this,_1,_2),
      "get timing information from last plan");
   RegisterCommand("GetTimes",
      boost::bind(&or_lemur::FamilyPlanner::GetTimes,this,_1,_2),
      "get timing information from last plan");
}

or_lemur::FamilyPlanner::~FamilyPlanner()
{
}

bool
or_lemur::FamilyPlanner::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & params_ser)
{
   or_lemur::FamilyParametersPtr params(new or_lemur::FamilyParameters());
   params_ser >> *params;
   params->Validate();
   return this->InitPlan(robot, params);
}

bool
or_lemur::FamilyPlanner::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params_base)
{
   if (!robot || !params_base)
   {
      RAVELOG_ERROR("robot/params objects must be passed!\n");
      return false;
   }
   FamilyParametersConstPtr params = boost::dynamic_pointer_cast<or_lemur::FamilyParameters const>(params_base);
   if (!params)
   {
      std::stringstream params_ser;
      params_ser << *params_base;
      return InitPlan(robot, params_ser);
   }
   
   // certainly remove any existing plan
   if (_current_family)
      _current_family->ompl_planner->setProblemDefinition(ompl::base::ProblemDefinitionPtr());
   
   // step 1: compute new_family (may be _current_family if it exists)
   // also store shared_ptr to the family module (we used it later)
   boost::shared_ptr<or_lemur::FamilyModule> mod_family;
   if (!_current_family)
   {
      boost::shared_ptr<CurrentFamily> new_family(new CurrentFamily);
      
      // find the family module
      if (!params->has_family_module)
      {
         RAVELOG_ERROR("No family module passed!\n");
         return false;
      }
      std::list<OpenRAVE::ModuleBasePtr> env_modules;
      GetEnv()->GetModules(env_modules);
      for (std::list<OpenRAVE::ModuleBasePtr>::iterator
         it=env_modules.begin(); it!=env_modules.end(); it++)
      {
         boost::shared_ptr<or_lemur::FamilyModule> mod
            = boost::dynamic_pointer_cast<or_lemur::FamilyModule>(*it);
         if (!mod)
            continue;
         if (mod->GetInstanceId() != params->family_module)
            continue;
         mod_family = mod;
         break;
      }
      if (!mod_family)
      {
         RAVELOG_ERROR("No matching family module passed!\n");
         return false;
      }
      new_family->mod_family = mod_family; // set weak_ptr
      
      // set robot data
      new_family->robot = robot;
      new_family->active_dofs = robot->GetActiveDOFIndices();
      
      // construct ompl space
      new_family->ompl_space.reset(new ompl::base::RealVectorStateSpace(new_family->active_dofs.size()));
      new_family->ompl_space->as<ompl::base::RealVectorStateSpace>()->setBounds(ompl_bounds(robot));
      new_family->ompl_space->setLongestValidSegmentFraction(ompl_resolution(robot) / new_family->ompl_space->getMaximumExtent());
      new_family->ompl_space->setup();
      
      // begin working on spaceinfo
      new_family->ompl_si.reset(new ompl::base::SpaceInformation(new_family->ompl_space));
      
      // get current family from mod_family
      // also canonical names
      new_family->familyspec = mod_family->GetCurrentFamily();
      new_family->familyspec_names = mod_family->GetCanonicalNames(new_family->familyspec);
      
      // construct ompl_family from familyspec
      new_family->ompl_family.reset(new ompl_lemur::Family);
      
      // convert each subset
      // for now, use an aborting si with a bogus cost
      // (we build a new checker during PlanPath() only!)
      for (std::set<or_lemur::FamilyModule::SetPtr>::iterator
         it=new_family->familyspec.sets.begin(); it!=new_family->familyspec.sets.end(); it++)
      {
         new_family->ompl_family->sets.insert(new_family->familyspec_names[*it]);
      }
      
      // convert relations
      for (std::set<or_lemur::FamilyModule::Relation>::iterator
         it=new_family->familyspec.relations.begin(); it!=new_family->familyspec.relations.end(); it++)
      {
         std::set<std::string> antecedents;
         for (std::set<or_lemur::FamilyModule::SetPtr>::iterator
            sit=it->first.begin(); sit!=it->first.end(); sit++)
         {
            antecedents.insert(new_family->familyspec_names[*sit]);
         }
         new_family->ompl_family->relations.insert(std::make_pair(antecedents,new_family->familyspec_names[it->second]));
      }
      
      // create the family effort model
      // (note -- we haven't yet set the target set!)
      new_family->ompl_family_checker.reset(new ompl_lemur::FamilyUtilityChecker(
         new_family->ompl_si.get(), *new_family->ompl_family));
      //fem->set_target(family->subsets.find("targ")->second.si);
      
      new_family->ompl_si->setStateValidityChecker(
         ompl::base::StateValidityCheckerPtr(new_family->ompl_family_checker));
      new_family->ompl_si->setup();
      
      // punt on tag cache for now
      new_family->ompl_tag_cache.reset(new ompl_lemur::DummyTagCache<ompl_lemur::LEMUR::VIdxTagMap,ompl_lemur::LEMUR::EIdxTagsMap>());
      
      new_family->ompl_planner.reset(new ompl_lemur::LEMUR(new_family->ompl_si, *new_family->ompl_tag_cache));
      
      new_family->ompl_planner->registerRoadmapType<ompl_lemur::RoadmapAAGrid>("AAGrid");
      new_family->ompl_planner->registerRoadmapType<ompl_lemur::RoadmapFromFile>("FromFile");
      new_family->ompl_planner->registerRoadmapType<ompl_lemur::RoadmapHalton>("Halton");
      new_family->ompl_planner->registerRoadmapType<ompl_lemur::RoadmapHaltonDens>("HaltonDens");
      new_family->ompl_planner->registerRoadmapType<ompl_lemur::RoadmapHaltonOffDens>("HaltonOffDens");
      new_family->ompl_planner->registerRoadmapType<ompl_lemur::RoadmapRGG>("RGG");
      new_family->ompl_planner->registerRoadmapType<ompl_lemur::RoadmapRGGDens>("RGGDens");
      new_family->ompl_planner->registerRoadmapType<ompl_lemur::RoadmapRGGDensConst>("RGGDensConst");
      new_family->ompl_planner->registerRoadmapType("CachedAAGrid",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapAAGrid>()));
      new_family->ompl_planner->registerRoadmapType("CachedHalton",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapHalton>()));
      new_family->ompl_planner->registerRoadmapType("CachedHaltonDens",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapHaltonDens>()));
      new_family->ompl_planner->registerRoadmapType("CachedHaltonOffDens",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapHaltonOffDens>()));
      new_family->ompl_planner->registerRoadmapType("CachedRGG",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapRGG>()));
      new_family->ompl_planner->registerRoadmapType("CachedRGGDens",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapRGGDens>()));
      new_family->ompl_planner->registerRoadmapType("CachedRGGDensConst",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapRGGDensConst>()));
      
      // great, everything initialized properly!
      _current_family = new_family;
   }
   else // validate _current_family (don't mutate it)!
   {
      // ensure family object is correct (save into mod_family)
      mod_family = _current_family->mod_family.lock();
      if (!mod_family)
      {
         RAVELOG_ERROR("Family module attached to current family no longer exists!\n");
         return false;
      }
      
      // if inparams references a module, ensure it's the same
      if (params->has_family_module)
      {
         if (mod_family->GetInstanceId() != params->family_module)
         {
            RAVELOG_ERROR("Requested family module doesn't match existing module!\n");
            return false;
         }
      }
      
      // ensure robot/activedofs is the same
      if (_current_family->robot.lock() != robot)
      {
         RAVELOG_ERROR("Family's robot doesn't match passed robot!\n");
         return false;
      }
      if (_current_family->active_dofs != robot->GetActiveDOFIndices())
      {
         RAVELOG_ERROR("Family's active dofs doesn't match passed active dofs!\n");
         return false;
      }
      
      // TODO: validate space settings (bounds, seg fraction, etc)
      
      // ensure family spec is the same
      // TODO: handle different family specs!
      or_lemur::FamilyModule::Family familyspec = mod_family->GetCurrentFamily();
      if (familyspec.sets != _current_family->familyspec.sets
         || familyspec.relations != _current_family->familyspec.relations)
      {
         RAVELOG_ERROR("Family specification is different!\n");
         RAVELOG_ERROR("(we don't yet support changing families)\n");
         return false;
      }
      
      // validate other stuff?
   }
   
   // planner params
   if (params->has_roadmap_type)
      _current_family->ompl_planner->setRoadmapType(params->roadmap_type);
   for (unsigned int ui=0; ui<params->roadmap_params.size(); ui++)
      _current_family->ompl_planner->params().setParam("roadmap."+params->roadmap_params[ui].first, params->roadmap_params[ui].second);
   if (params->has_coeff_distance)
      _current_family->ompl_planner->setCoeffDistance(params->coeff_distance);
   if (params->has_coeff_checkcost)
      _current_family->ompl_planner->setCoeffCheckcost(params->coeff_checkcost);
   if (params->has_coeff_batch)
      _current_family->ompl_planner->setCoeffBatch(params->coeff_batch);
   if (params->has_do_timing)
      _current_family->ompl_planner->setDoTiming(params->do_timing);
   if (params->has_persist_roots)
      _current_family->ompl_planner->setPersistRoots(params->persist_roots);
   if (params->has_num_batches_init)
      _current_family->ompl_planner->setNumBatchesInit(params->num_batches_init);
   if (params->has_max_batches)
      _current_family->ompl_planner->setMaxBatches(params->max_batches);
   if (params->has_search_type)
      _current_family->ompl_planner->setSearchType(params->search_type);
   if (params->has_eval_type)
      _current_family->ompl_planner->setEvalType(params->eval_type);
   
   // problem definition
   ompl::base::ProblemDefinitionPtr ompl_pdef(
      new ompl::base::ProblemDefinition(_current_family->ompl_si));
   ompl_set_roots(ompl_pdef, params);
   _current_family->ompl_planner->setProblemDefinition(ompl_pdef);
   
   _current_family->params_last = params;
   return true;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr
or_lemur::FamilyPlanner::GetParameters() const
{
   or_lemur::FamilyParametersPtr params(new or_lemur::FamilyParameters());
   
   // get params from ompl_planner if it exists!
   
   return params;
}

OpenRAVE::PlannerStatus
or_lemur::FamilyPlanner::PlanPath(OpenRAVE::TrajectoryBasePtr traj)
{
   // get current family
   boost::shared_ptr<CurrentFamily> current_family = _current_family;
   if (!current_family)
      throw OpenRAVE::openrave_exception("Plan not initialized!");

   // get robot
   OpenRAVE::RobotBasePtr robot = current_family->robot.lock();
   if (!robot)
      throw OpenRAVE::openrave_exception("was InitPlan called, or was robot removed from env?");

   // get family module
   boost::shared_ptr<or_lemur::FamilyModule> mod_family = current_family->mod_family.lock();
   if (!mod_family)
      throw OpenRAVE::openrave_exception("Family module no longer exists?");
   
   // get current set
   or_lemur::FamilyModule::SetPtr current_set = mod_family->GetCurrentSet();
   std::string current_set_name = current_family->familyspec_names[current_set];
   RAVELOG_INFO("planning in target set \"%s\" ...\n", current_set_name.c_str());
   
   // get indicators from or family
   std::map< or_lemur::FamilyModule::SetPtr, std::pair<double,or_lemur::FamilyModule::Indicator> >
      indicators = mod_family->GetIndicators(current_family->familyspec);
   
   // construct an ompl SetChecker for each set
   std::map<std::string, ompl_lemur::FamilyUtilityChecker::SetChecker> set_checkers;
   
   for (std::set<or_lemur::FamilyModule::SetPtr>::iterator
      it=current_family->familyspec.sets.begin(); it!=current_family->familyspec.sets.end(); it++)
   {
      ompl::base::StateValidityCheckerPtr checker(new OrIndicatorChecker(
         current_family->ompl_si, indicators[*it].second));
      set_checkers.insert(std::make_pair(
         current_family->familyspec_names[*it],
         std::make_pair(indicators[*it].first,checker)));
   }
   
   // start checking
   current_family->ompl_family_checker->start_checking(current_set_name, set_checkers);
   
   printf("planning ...\n");
   
#if 0
   ompl_checker->num_checks = 0;
   ompl_checker->dur_checks = boost::chrono::high_resolution_clock::duration();
#endif

   std::ofstream fp_alglog;
   if (current_family->params_last->alglog == "-")
   {
      current_family->ompl_planner->as<ompl_lemur::LEMUR>()->os_alglog = &std::cout;
   }
   else if (current_family->params_last->alglog != "")
   {
      if (current_family->params_last->has_do_alglog_append && current_family->params_last->do_alglog_append)
         fp_alglog.open(current_family->params_last->alglog.c_str(), std::ios_base::app);
      else
         fp_alglog.open(current_family->params_last->alglog.c_str(), std::ios_base::out);
      current_family->ompl_planner->as<ompl_lemur::LEMUR>()->os_alglog = &fp_alglog;
   }
   
   ompl::base::PlannerStatus ompl_status;
   ompl::base::PlannerTerminationCondition ptc(ompl::base::plannerNonTerminatingCondition());
   if (current_family->params_last->has_time_limit)
      ptc = ompl::base::timedPlannerTerminationCondition(current_family->params_last->time_limit);
   ompl_status = current_family->ompl_planner->solve(ptc);
   printf("planner returned: %s\n", ompl_status.asString().c_str());

   if (current_family->params_last->has_do_roadmap_save && current_family->params_last->do_roadmap_save)
   {
      boost::shared_ptr< or_lemur::RoadmapCached<ompl_lemur::LEMUR::RoadmapArgs> > cached_roadmap
         = boost::dynamic_pointer_cast< or_lemur::RoadmapCached<ompl_lemur::LEMUR::RoadmapArgs> >(current_family->ompl_planner->_roadmap);
      if (cached_roadmap)
      {
         printf("saving cached roadmap ...\n");
         current_family->ompl_planner->_roadmap->as< or_lemur::RoadmapCached<ompl_lemur::LEMUR::RoadmapArgs> >()->save_file();
      }
      else
      {
         current_family->ompl_family_checker->stop_checking();
         throw OpenRAVE::openrave_exception("asked to save roadmap cache, but non-cached roadmap used!");
      }
   }

   current_family->ompl_planner->as<ompl_lemur::LEMUR>()->os_alglog = 0;
   fp_alglog.close();
   
   if (current_family->params_last->graph == "-")
   {
      current_family->ompl_planner->as<ompl_lemur::LEMUR>()->dump_graph(std::cout);
   }
   else if (current_family->params_last->graph != "")
   {
      std::ofstream fp_graph;
      fp_graph.open(current_family->params_last->graph.c_str());
      current_family->ompl_planner->as<ompl_lemur::LEMUR>()->dump_graph(fp_graph);
      fp_graph.close();
   }
   
   if (ompl_status != ompl::base::PlannerStatus::EXACT_SOLUTION)
   {
      current_family->ompl_family_checker->stop_checking();
      return OpenRAVE::PS_Failed;
   }
   
   // convert result
   // (if the planner exited with an exact empty solution, then it's done!)
   ompl::base::PathPtr path = current_family->ompl_planner->getProblemDefinition()->getSolutionPath();
   if (!path)
   {
      current_family->ompl_family_checker->stop_checking();
      return OpenRAVE::PS_Failed;
   }
   
   ompl::geometric::PathGeometric * gpath = dynamic_cast<ompl::geometric::PathGeometric*>(path.get());
   if (!gpath)
   {
      current_family->ompl_family_checker->stop_checking();
      throw OpenRAVE::openrave_exception("ompl path is not geometric for some reason!");
   }
   traj->Init(robot->GetActiveConfigurationSpecification());
   for (unsigned int i=0; i<gpath->getStateCount(); i++)
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(current_family->ompl_space, gpath->getState(i));
      traj->Insert(i, std::vector<OpenRAVE::dReal>(&s[0], &s[0]+robot->GetActiveDOF()));
   }
   
   current_family->ompl_family_checker->stop_checking();
   return OpenRAVE::PS_HasSolution;
}

bool or_lemur::FamilyPlanner::ResetFamily(std::ostream & sout, std::istream & sin)
{
   _current_family.reset();
   return true;
}

bool or_lemur::FamilyPlanner::GetTimes(std::ostream & sout, std::istream & sin) const
{
   // get current family
   boost::shared_ptr<CurrentFamily> current_family = _current_family;
   if (!current_family)
      throw OpenRAVE::openrave_exception("Plan not initialized!");
#if 0
   //sout << "checktime " << boost::chrono::duration<double>(ompl_checker->dur_checks).count();
   //sout << " totaltime " << 0.0;
   if (family)
   {
      for (std::map<std::string, ompl_lemur::Family::Subset>::iterator
         it=family->subsets.begin(); it!=family->subsets.end(); it++)
      {
         sout << " n_checks_" << it->first;
         sout << " " << ((OrIndicatorChecker*)it->second.si->getStateValidityChecker().get())->num_checks;
      }
   }
#endif
   sout << " e8_dur_total " << current_family->ompl_planner->as<ompl_lemur::LEMUR>()->getDurTotal();
   sout << " e8_dur_roadmapgen " <<  current_family->ompl_planner->as<ompl_lemur::LEMUR>()->getDurRoadmapGen();
   sout << " e8_dur_roadmapinit " <<  current_family->ompl_planner->as<ompl_lemur::LEMUR>()->getDurRoadmapInit();
   sout << " e8_dur_lazysp " <<  current_family->ompl_planner->as<ompl_lemur::LEMUR>()->getDurLazySP();
   sout << " e8_dur_search " <<  current_family->ompl_planner->as<ompl_lemur::LEMUR>()->getDurSearch();
   sout << " e8_dur_eval " <<  current_family->ompl_planner->as<ompl_lemur::LEMUR>()->getDurEval();
   sout << " e8_dur_selector_init " <<  current_family->ompl_planner->as<ompl_lemur::LEMUR>()->getDurSelectorInit();
   sout << " e8_dur_selector " <<  current_family->ompl_planner->as<ompl_lemur::LEMUR>()->getDurSelector();
   sout << " e8_dur_selector_notify " <<  current_family->ompl_planner->as<ompl_lemur::LEMUR>()->getDurSelectorNotify();
   return true;
}
