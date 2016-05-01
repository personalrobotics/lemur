/*! \file planner_family.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
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
#include <pr_bgl/vector_ref_property_map.h>
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
#include <ompl_lemur/FamilyTagCache.h>
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
#include <or_lemur/or_ompl_conversions.h>
#include <or_lemur/params_lemur.h>
#include <or_lemur/params_family.h>
#include <or_lemur/module_family.h>
#include <or_lemur/planner_family.h>


or_lemur::FamilyPlanner::FamilyPlanner(OpenRAVE::EnvironmentBasePtr env):
   OpenRAVE::PlannerBase(env), _initialized(false)
{
   __description = "Family Planner (based on LEMUR)";
   RegisterCommand("SaveSetCaches",
      boost::bind(&or_lemur::FamilyPlanner::CmdSaveSetCaches,this,_1,_2),
      "SaveSetCaches");
   RegisterCommand("ResetFamily",
      boost::bind(&or_lemur::FamilyPlanner::CmdResetFamily,this,_1,_2),
      "ResetFamily");
   RegisterCommand("GetTimes",
      boost::bind(&or_lemur::FamilyPlanner::CmdGetTimes,this,_1,_2),
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
   return InitPlan(robot, params);
}

bool
or_lemur::FamilyPlanner::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params_base)
{
   _initialized = false;
   
   if (!robot || !params_base)
   {
      RAVELOG_ERROR("Robot/params objects must be passed!\n");
      return false;
   }
   FamilyParametersConstPtr params = boost::dynamic_pointer_cast<or_lemur::FamilyParameters const>(params_base);
   if (!params)
   {
      std::stringstream params_ser;
      params_ser << *params_base;
      return InitPlan(robot, params_ser);
   }
   
   // step 1: compute new_family (may be _current_family if it exists)
   // also store shared_ptr to the family module (we used it later)
   boost::shared_ptr<or_lemur::FamilyModule> mod_family;
   if (!_current_family)
   {
      boost::shared_ptr<CurrentFamily> fam(new CurrentFamily);
      
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
      fam->mod_family = mod_family; // set weak_ptr
      
      // set robot data
      fam->robot = robot;
      fam->active_dofs = robot->GetActiveDOFIndices();
      
      // construct ompl spaceinfo (with no checker)
      bool success = or_lemur::create_space(robot, fam->active_dofs, false, false, fam->ompl_si);
      if (!success)
         return false;
      
      // initialize setcaches
      for (unsigned int ui=0; ui<params->family_setcaches.size(); ui++)
      {
         SetCache set_cache;
         set_cache.name = params->family_setcaches[ui].name; // may be empty
         set_cache.filename = params->family_setcaches[ui].filename;
         RAVELOG_INFO("Passed family cache name: |%s| filename: |%s|\n",
            set_cache.name.c_str(),
            set_cache.filename.c_str());
         
         if (set_cache.filename == "")
         {
            RAVELOG_ERROR("SetCache filename must be passed!\n");
            return false;
         }
         
         // does file already exist?
         // if so, parse file header into set header, roadmap header
         // and get the set from the family module
         std::ifstream fp(set_cache.filename.c_str());
         if (fp.is_open())
         {
            std::string line;
            
            // read into set_header
            while (std::getline(fp, line))
            {
               if (line == "---")
                  break;
               set_cache.set_header += line + "\n";
            }
            
            // read roadmap_header
            // (we will check consistency with the planner's roadmap type later)
            while (std::getline(fp, line))
            {
               if (line == "---")
                  break;
               set_cache.roadmap_header += line + "\n";
            }
            
            // ensure we actually read a roadmap header
            if (!set_cache.roadmap_header.size())
            {
               RAVELOG_ERROR("SetCache file %s does not contain a roadmap header!\n",
                  set_cache.filename.c_str());
               return false;
            }
            
            // retrieve the set from the family by header
            set_cache.set = mod_family->GetSetFromHeader(set_cache.set_header);
            if (!set_cache.set)
            {
               RAVELOG_ERROR("SetCache file %s loading failed!\n",
                  set_cache.filename.c_str());
               return false;
            }
            
            if (set_cache.name != "")
            {
               FamilyModule::SetPtr named_set = mod_family->GetSet(set_cache.name);
               if (named_set && named_set != set_cache.set)
               {
                  RAVELOG_ERROR("SetCache file %s refers to different set than named set \"%s\"!\n",
                     set_cache.filename.c_str(), set_cache.name.c_str());
                  return false;
               }
            }
         }
         else
         {
            // otherwise, the set definition comes from the family module by name!

            // if no file, then the name must exist!
            if (set_cache.name == "")
            {
               RAVELOG_ERROR("SetCache file %s does not exist, and no name specified for lookup!\n",
                  set_cache.filename.c_str());
               return false;
            }
            
            // find the set with the matching name
            set_cache.set = mod_family->GetSet(set_cache.name);
            if (!set_cache.set)
            {
               RAVELOG_ERROR("SetCache name %s does not exist in family!\n",
                  set_cache.name.c_str());
               return false;
            }
            
            // ok, we have a set! get header ...
            set_cache.set_header = mod_family->GetHeaderFromSet(set_cache.set);
            
            // roadmap_header will be filled later!
         }
         
         fam->setcaches.insert(std::make_pair(set_cache.filename,set_cache));
      }
      
      // get current set (that we'll plan in)
      fam->set_current = mod_family->GetCurrentSet();
      
      // get current family from mod_family
      // this must include the live plan,
      // and also any sets our setcaches reference
      std::set<FamilyModule::SetPtr> sets;
      sets.insert(fam->set_current);
      for (std::map<std::string, SetCache>::iterator
         it=fam->setcaches.begin(); it!=fam->setcaches.end(); it++)
      {
         sets.insert(it->second.set);
      }
      
      // also canonical names
      // TODO: make family and setcaches dynamic!
      fam->familyspec = mod_family->GetCurrentFamily(sets);
      fam->familyspec_names = mod_family->GetCanonicalNames(fam->familyspec);
      // replace names for any sets we have setcaches for
      for (std::map<std::string, SetCache>::iterator
         it=fam->setcaches.begin(); it!=fam->setcaches.end(); it++)
      {
         if (it->second.name != "")
            fam->familyspec_names[it->second.set] = it->second.name;
      }
      
      // construct ompl_family from familyspec
      fam->ompl_family.reset(new ompl_lemur::Family);
      
      // convert each subset
      // for now, use an aborting si with a bogus cost
      // (we build a new checker during PlanPath() only!)
      for (std::set<or_lemur::FamilyModule::SetPtr>::iterator
         it=fam->familyspec.sets.begin(); it!=fam->familyspec.sets.end(); it++)
      {
         fam->ompl_family->sets.insert(fam->familyspec_names[*it]);
      }
      
      // convert relations
      for (std::set<or_lemur::FamilyModule::Relation>::iterator
         it=fam->familyspec.relations.begin(); it!=fam->familyspec.relations.end(); it++)
      {
         std::set<std::string> antecedents;
         for (std::set<or_lemur::FamilyModule::SetPtr>::iterator
            sit=it->first.begin(); sit!=it->first.end(); sit++)
         {
            antecedents.insert(fam->familyspec_names[*sit]);
         }
         fam->ompl_family->relations.insert(std::make_pair(antecedents,fam->familyspec_names[it->second]));
      }
      
      // create the family effort model
      // (note -- we haven't yet set the target set!)
      fam->ompl_family_checker.reset(new ompl_lemur::FamilyUtilityChecker(
         fam->ompl_si.get(), *fam->ompl_family));
      
      fam->ompl_si->setStateValidityChecker(
         ompl::base::StateValidityCheckerPtr(fam->ompl_family_checker));
      fam->ompl_si->setup();
      
      // create planner
      fam->ompl_lemur.reset(new ompl_lemur::LEMUR(fam->ompl_si));
      
      // create family tag cache object
      fam->ompl_tag_cache.reset(new ompl_lemur::FamilyTagCache<ompl_lemur::LEMUR::VIdxTagMap,ompl_lemur::LEMUR::EIdxTagsMap>(fam->ompl_family_checker));
      fam->ompl_lemur->_tag_cache = fam->ompl_tag_cache;
      
      fam->ompl_lemur->registerRoadmapType<ompl_lemur::RoadmapAAGrid>("AAGrid");
      fam->ompl_lemur->registerRoadmapType<ompl_lemur::RoadmapFromFile>("FromFile");
      fam->ompl_lemur->registerRoadmapType<ompl_lemur::RoadmapHalton>("Halton");
      fam->ompl_lemur->registerRoadmapType<ompl_lemur::RoadmapHaltonDens>("HaltonDens");
      fam->ompl_lemur->registerRoadmapType<ompl_lemur::RoadmapHaltonOffDens>("HaltonOffDens");
      fam->ompl_lemur->registerRoadmapType<ompl_lemur::RoadmapRGG>("RGG");
      fam->ompl_lemur->registerRoadmapType<ompl_lemur::RoadmapRGGDens>("RGGDens");
      fam->ompl_lemur->registerRoadmapType<ompl_lemur::RoadmapRGGDensConst>("RGGDensConst");
      fam->ompl_lemur->registerRoadmapType("CachedAAGrid",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapAAGrid>()));
      fam->ompl_lemur->registerRoadmapType("CachedHalton",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapHalton>()));
      fam->ompl_lemur->registerRoadmapType("CachedHaltonDens",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapHaltonDens>()));
      fam->ompl_lemur->registerRoadmapType("CachedHaltonOffDens",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapHaltonOffDens>()));
      fam->ompl_lemur->registerRoadmapType("CachedRGG",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapRGG>()));
      fam->ompl_lemur->registerRoadmapType("CachedRGGDens",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapRGGDens>()));
      fam->ompl_lemur->registerRoadmapType("CachedRGGDensConst",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapRGGDensConst>()));
      
      // great, everything initialized properly!
      _current_family = fam;
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
      _current_family->ompl_lemur->setRoadmapType(params->roadmap_type);
   for (unsigned int ui=0; ui<params->roadmap_params.size(); ui++)
      _current_family->ompl_lemur->params().setParam("roadmap."+params->roadmap_params[ui].first, params->roadmap_params[ui].second);
   if (params->has_coeff_distance)
      _current_family->ompl_lemur->setCoeffDistance(params->coeff_distance);
   if (params->has_coeff_checkcost)
      _current_family->ompl_lemur->setCoeffCheckcost(params->coeff_checkcost);
   if (params->has_coeff_batch)
      _current_family->ompl_lemur->setCoeffBatch(params->coeff_batch);
   if (params->has_do_timing)
      _current_family->ompl_lemur->setDoTiming(params->do_timing);
   if (params->has_persist_roots)
      _current_family->ompl_lemur->setPersistRoots(params->persist_roots);
   if (params->has_num_batches_init)
      _current_family->ompl_lemur->setNumBatchesInit(params->num_batches_init);
   if (params->has_max_batches)
      _current_family->ompl_lemur->setMaxBatches(params->max_batches);
   if (params->has_solve_all)
      _current_family->ompl_lemur->setSolveAll(params->solve_all);
   if (params->has_search_type)
      _current_family->ompl_lemur->setSearchType(params->search_type);
   if (params->has_eval_type)
      _current_family->ompl_lemur->setEvalType(params->eval_type);
   
   // reset current set
   _current_family->set_current = mod_family->GetCurrentSet();
   
   // problem definition
   // the user can either set initial/goal configs, OR do_solve_all
   ompl::base::ProblemDefinitionPtr ompl_pdef(
      new ompl::base::ProblemDefinition(_current_family->ompl_si));
   bool success = ompl_set_roots(ompl_pdef, params);
   if (!success)
      return false;
   _current_family->ompl_lemur->setProblemDefinition(ompl_pdef);
   
   // now that a problem definition is set, we can trust the roadmap id!
   if (_current_family->roadmap_header == "")
   {
      _current_family->roadmap_header = ompl_lemur::roadmap_id(
         _current_family->ompl_lemur->getRoadmap().get()) + "\n";
      // update / validate roadmap ids for setacaches
      for (std::map<std::string, SetCache>::iterator
         it=_current_family->setcaches.begin(); it!=_current_family->setcaches.end(); it++)
      {
         std::string planner_roadmap = _current_family->roadmap_header;
         if (planner_roadmap.substr(0,11) == "type=Cached")
            planner_roadmap = "type=" + planner_roadmap.substr(11);
         if (it->second.roadmap_header == "")
         {
            it->second.roadmap_header = planner_roadmap;
         }
         else if (it->second.roadmap_header != planner_roadmap)
         {
            RAVELOG_ERROR("setcache roadmap header mismatch!\n");
            RAVELOG_ERROR(" planner roadmap: %s", planner_roadmap.c_str());
            RAVELOG_ERROR("setcache roadmap: %s", it->second.roadmap_header.c_str());
            return false;
         }
         
         // reconcile setcaches with now-known roadmap header!
         // TODO: what it setcaches change between InitPlan calls?
         std::string file_header;
         file_header += it->second.set_header;
         file_header += "---\n";
         file_header += it->second.roadmap_header;
         file_header += "---\n";
         _current_family->ompl_tag_cache->addCachedSet(_current_family->familyspec_names[it->second.set], it->second.filename, file_header);
      }
   }
   
   _current_family->params_last = params;
   _initialized = true;
   return true;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr
or_lemur::FamilyPlanner::GetParameters() const
{
   or_lemur::FamilyParametersPtr params(new or_lemur::FamilyParameters());
   RAVELOG_WARN("GetParameters() not yet implemented!\n");
   return params;
}

OpenRAVE::PlannerStatus
or_lemur::FamilyPlanner::PlanPath(OpenRAVE::TrajectoryBasePtr traj)
{
   // get current family
   boost::shared_ptr<CurrentFamily> fam = _current_family;
   if (!_initialized || !fam)
      throw OpenRAVE::openrave_exception("Plan not initialized!");

   // get robot
   OpenRAVE::RobotBasePtr robot = fam->robot.lock();
   if (!robot)
      throw OpenRAVE::openrave_exception("Was InitPlan called, or was robot removed from env?");

   // get family module
   boost::shared_ptr<or_lemur::FamilyModule> mod_family = fam->mod_family.lock();
   if (!mod_family)
      throw OpenRAVE::openrave_exception("Family module no longer exists?");
   
   // get current set
   std::string current_set_name = fam->familyspec_names[fam->set_current];
   RAVELOG_INFO("Planning in target set \"%s\" ...\n", current_set_name.c_str());
   
   // get current live indicators from or family
   std::map< or_lemur::FamilyModule::SetPtr, std::pair<double,or_lemur::FamilyModule::Indicator> >
      indicators = mod_family->GetIndicators(fam->familyspec);
   
   // construct an ompl SetChecker for each set
   std::map<std::string, ompl_lemur::FamilyUtilityChecker::SetChecker> set_checkers;
   
   for (std::set<or_lemur::FamilyModule::SetPtr>::iterator
      it=fam->familyspec.sets.begin(); it!=fam->familyspec.sets.end(); it++)
   {
      ompl::base::StateValidityCheckerPtr checker(new OrIndicatorChecker(
         fam->ompl_si, indicators[*it].second));
      set_checkers.insert(std::make_pair(
         fam->familyspec_names[*it],
         std::make_pair(indicators[*it].first,checker)));
   }
   
   // start checking
   fam->ompl_family_checker->start_checking(current_set_name, set_checkers);
   
#if 0
   ompl_checker->num_checks = 0;
   ompl_checker->dur_checks = boost::chrono::high_resolution_clock::duration();
#endif

   std::ofstream fp_alglog;
   if (fam->params_last->alglog == "-")
   {
      fam->ompl_lemur->os_alglog = &std::cout;
   }
   else if (fam->params_last->alglog != "")
   {
      if (fam->params_last->has_do_alglog_append && fam->params_last->do_alglog_append)
         fp_alglog.open(fam->params_last->alglog.c_str(), std::ios_base::app);
      else
         fp_alglog.open(fam->params_last->alglog.c_str(), std::ios_base::out);
      fam->ompl_lemur->os_alglog = &fp_alglog;
   }
   
   ompl::base::PlannerStatus ompl_status;
   ompl::base::PlannerTerminationCondition ptc(ompl::base::plannerNonTerminatingCondition());
   if (fam->params_last->has_time_limit)
      ptc = ompl::base::timedPlannerTerminationCondition(fam->params_last->time_limit);
   ompl_status = fam->ompl_lemur->solve(ptc);

   if (fam->params_last->has_do_roadmap_save && fam->params_last->do_roadmap_save)
   {
      boost::shared_ptr< const or_lemur::RoadmapCached<ompl_lemur::LEMUR::RoadmapArgs> > cached_roadmap
         = boost::dynamic_pointer_cast< const or_lemur::RoadmapCached<ompl_lemur::LEMUR::RoadmapArgs> >(fam->ompl_lemur->getRoadmap());
      if (cached_roadmap)
      {
         RAVELOG_INFO("Saving cached roadmap ...\n");
         cached_roadmap->save_file();
      }
      else
      {
         fam->ompl_family_checker->stop_checking();
         throw OpenRAVE::openrave_exception("Asked to save roadmap cache, but non-cached roadmap used.");
      }
   }

   fam->ompl_lemur->os_alglog = 0;
   fp_alglog.close();
   
   if (fam->params_last->graph == "-")
   {
      fam->ompl_lemur->dump_graph(std::cout);
   }
   else if (fam->params_last->graph != "")
   {
      std::ofstream fp_graph;
      fp_graph.open(fam->params_last->graph.c_str());
      fam->ompl_lemur->dump_graph(fp_graph);
      fp_graph.close();
   }
   
   if (ompl_status != ompl::base::PlannerStatus::EXACT_SOLUTION)
   {
      fam->ompl_family_checker->stop_checking();
      return OpenRAVE::PS_Failed;
   }
   
   if (fam->params_last->has_solve_all && fam->params_last->solve_all)
   {
      fam->ompl_family_checker->stop_checking();
      if (traj)
         traj->Init(robot->GetActiveConfigurationSpecification()); // reset traj
      return OpenRAVE::PS_HasSolution;
   }
   
   // convert result
   // (if the planner exited with an exact empty solution, then it's done!)
   ompl::base::PathPtr path = fam->ompl_lemur->getProblemDefinition()->getSolutionPath();
   if (!path)
   {
      fam->ompl_family_checker->stop_checking();
      return OpenRAVE::PS_Failed;
   }
   
   ompl::geometric::PathGeometric * gpath = dynamic_cast<ompl::geometric::PathGeometric*>(path.get());
   if (!gpath)
   {
      fam->ompl_family_checker->stop_checking();
      throw OpenRAVE::openrave_exception("OMPL path is not geometric for some reason.");
   }
   traj->Init(robot->GetActiveConfigurationSpecification());
   for (unsigned int i=0; i<gpath->getStateCount(); i++)
   {
      std::vector<double> values;
      fam->ompl_si->getStateSpace()->copyToReals(values, gpath->getState(i));
      // TODO: this fails if openrave was compiled with floats!
      traj->Insert(i, values);
   }
   
   fam->ompl_family_checker->stop_checking();
   return OpenRAVE::PS_HasSolution;
}

bool or_lemur::FamilyPlanner::CmdSaveSetCaches(std::ostream & sout, std::istream & sin)
{
   // get current family
   boost::shared_ptr<CurrentFamily> fam = _current_family;
   if (!_initialized || !fam)
      throw OpenRAVE::openrave_exception("Plan not initialized!");
   
   RAVELOG_INFO("SaveSetCaches called.\n");
   
   fam->ompl_lemur->saveTagCache();
   
   return true;
}

bool or_lemur::FamilyPlanner::CmdResetFamily(std::ostream & sout, std::istream & sin)
{
   _current_family.reset();
   return true;
}

bool or_lemur::FamilyPlanner::CmdGetTimes(std::ostream & sout, std::istream & sin) const
{
   // get current family
   boost::shared_ptr<CurrentFamily> fam = _current_family;
   if (!_initialized || !fam)
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
   sout << " dur_total " << fam->ompl_lemur->getDurTotal();
   sout << " dur_roadmapgen " <<  fam->ompl_lemur->getDurRoadmapGen();
   sout << " dur_roadmapinit " <<  fam->ompl_lemur->getDurRoadmapInit();
   sout << " dur_lazysp " <<  fam->ompl_lemur->getDurLazySP();
   sout << " dur_search " <<  fam->ompl_lemur->getDurSearch();
   sout << " dur_eval " <<  fam->ompl_lemur->getDurEval();
   sout << " dur_selector_init " <<  fam->ompl_lemur->getDurSelectorInit();
   sout << " dur_selector " <<  fam->ompl_lemur->getDurSelector();
   sout << " dur_selector_notify " <<  fam->ompl_lemur->getDurSelectorNotify();
   return true;
}
