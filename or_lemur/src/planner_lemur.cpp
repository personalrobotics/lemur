/* File: planner_lemur.cpp
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
#include <ompl_lemur/UtilityChecker.h>
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
#include <or_lemur/or_checker.h>
#include <or_lemur/params_lemur.h>
#include <or_lemur/planner_lemur.h>


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

} // anonymous namespace


or_lemur::LEMUR::LEMUR(OpenRAVE::EnvironmentBasePtr env):
   OpenRAVE::PlannerBase(env), env(env)
{
   __description = "E8 roadmap planner";
   RegisterCommand("GetTimes",
      boost::bind(&or_lemur::LEMUR::GetTimes,this,_1,_2),
      "get timing information from last plan");
}

or_lemur::LEMUR::~LEMUR()
{
}

bool
or_lemur::LEMUR::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & inparams_ser)
{
   or_lemur::LEMURParametersPtr inparams(new or_lemur::LEMURParameters());
   inparams_ser >> *inparams;
   inparams->Validate();
   return InitPlan(robot, inparams);
}

bool
or_lemur::LEMUR::InitPlan(OpenRAVE::RobotBasePtr inrobot, OpenRAVE::PlannerBase::PlannerParametersConstPtr inparams_base)
{
   LEMURParametersConstPtr inparams = boost::dynamic_pointer_cast<or_lemur::LEMURParameters const>(inparams_base);
   if (!inparams)
   {
      std::stringstream inparams_ser;
      inparams_ser << *inparams_base;
      return InitPlan(inrobot, inparams_ser);
   }
   if (!inrobot || !inparams_base)
      throw OpenRAVE::openrave_exception("robot/params objects must be passed!");
   
   // do setup (at most once)
   params = inparams;
   robot = inrobot;
   robot_adofs = inrobot->GetActiveDOFIndices();
   
   bool persist_planner = false;
   if (params->has_persist_roots && params->persist_roots)
      persist_planner = true;
   
   if (persist_planner)
      RAVELOG_WARN("Warning, persisting the planner is experimental and does not check for consistency!\n");
   
   if (!ompl_planner || !persist_planner)
   {
      // set up ompl space
      ompl_space.reset(new ompl::base::RealVectorStateSpace(robot_adofs.size()));
      ompl_space->as<ompl::base::RealVectorStateSpace>()->setBounds(ompl_bounds(robot));
      ompl_space->setLongestValidSegmentFraction(ompl_resolution(robot) / ompl_space->getMaximumExtent());
      ompl_space->setup();
      
      // set up si / checker
      ompl_si.reset(new ompl::base::SpaceInformation(ompl_space));
   }
   
   ompl_checker.reset(new or_lemur::OrChecker(ompl_si, env, robot, robot_adofs.size()));
   
   double check_cost;
   if (params->has_check_cost)
      check_cost = params->check_cost;
   else
   {
      check_cost = ompl_space->getLongestValidSegmentLength();
      printf("using simple effort model with default check_cost=%f\n", check_cost);
   }
   ompl_binary_checker.reset(new ompl_lemur::BinaryUtilityChecker(ompl_si, ompl_checker, check_cost));
   ompl_si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(ompl_binary_checker));
   ompl_si->setup();
   
   if (!ompl_planner || !persist_planner)
   {
      // set up planner
      tag_cache.reset(new ompl_lemur::DummyTagCache<ompl_lemur::LEMUR::VIdxTagMap,ompl_lemur::LEMUR::EIdxTagsMap>());
      ompl_planner.reset(new ompl_lemur::LEMUR(ompl_si, *tag_cache));
      
      // register known roadmap types
      ompl_planner->registerRoadmapType<ompl_lemur::RoadmapAAGrid>("AAGrid");
      ompl_planner->registerRoadmapType<ompl_lemur::RoadmapFromFile>("FromFile");
      ompl_planner->registerRoadmapType<ompl_lemur::RoadmapHalton>("Halton");
      ompl_planner->registerRoadmapType<ompl_lemur::RoadmapHaltonDens>("HaltonDens");
      ompl_planner->registerRoadmapType<ompl_lemur::RoadmapHaltonOffDens>("HaltonOffDens");
      ompl_planner->registerRoadmapType<ompl_lemur::RoadmapRGG>("RGG");
      ompl_planner->registerRoadmapType<ompl_lemur::RoadmapRGGDens>("RGGDens");
      ompl_planner->registerRoadmapType<ompl_lemur::RoadmapRGGDensConst>("RGGDensConst");
      ompl_planner->registerRoadmapType("CachedAAGrid",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapAAGrid>()));
      ompl_planner->registerRoadmapType("CachedHalton",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapHalton>()));
      ompl_planner->registerRoadmapType("CachedHaltonDens",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapHaltonDens>()));
      ompl_planner->registerRoadmapType("CachedHaltonOffDens",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapHaltonOffDens>()));
      ompl_planner->registerRoadmapType("CachedRGG",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapRGG>()));
      ompl_planner->registerRoadmapType("CachedRGGDens",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapRGGDens>()));
      ompl_planner->registerRoadmapType("CachedRGGDensConst",
         or_lemur::RoadmapCachedFactory<ompl_lemur::LEMUR::RoadmapArgs>(
            ompl_lemur::RoadmapFactory<ompl_lemur::LEMUR::RoadmapArgs,ompl_lemur::RoadmapRGGDensConst>()));
   }
   
   // planner params
   if (params->has_roadmap_type)
      ompl_planner->setRoadmapType(params->roadmap_type);
   for (unsigned int ui=0; ui<params->roadmap_params.size(); ui++)
      ompl_planner->params().setParam("roadmap."+params->roadmap_params[ui].first, params->roadmap_params[ui].second);
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
   
   // force reeval of wlazy
   ompl_binary_checker->_has_changed = true;
   
   // problem definition
   ompl_pdef.reset(new ompl::base::ProblemDefinition(ompl_si));
   ompl_set_roots(ompl_pdef, params);   
   ompl_planner->setProblemDefinition(ompl_pdef);
   
   return true;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr
or_lemur::LEMUR::GetParameters() const
{
   return params;
}

OpenRAVE::PlannerStatus
or_lemur::LEMUR::PlanPath(OpenRAVE::TrajectoryBasePtr traj)
{
   printf("planning ...\n");
   
   ompl_checker->num_checks = 0;
   ompl_checker->dur_checks = boost::chrono::high_resolution_clock::duration();
   
   std::ofstream fp_alglog;
   if (params->alglog == "-")
   {
      ompl_planner->as<ompl_lemur::LEMUR>()->os_alglog = &std::cout;
   }
   else if (params->has_alglog)
   {
      if (params->has_do_alglog_append && params->do_alglog_append)
         fp_alglog.open(params->alglog.c_str(), std::ios_base::app);
      else
         fp_alglog.open(params->alglog.c_str(), std::ios_base::out);
      ompl_planner->as<ompl_lemur::LEMUR>()->os_alglog = &fp_alglog;
   }
   
   ompl::base::PlannerStatus ompl_status;
   ompl::base::PlannerTerminationCondition ptc(ompl::base::plannerNonTerminatingCondition());
   if (params->has_time_limit)
      ptc = ompl::base::timedPlannerTerminationCondition(params->time_limit);
   ompl_status = ompl_planner->solve(ptc);
   printf("planner returned: %s\n", ompl_status.asString().c_str());
   
   if (params->has_do_roadmap_save && params->do_roadmap_save)
   {
      boost::shared_ptr< or_lemur::RoadmapCached<ompl_lemur::LEMUR::RoadmapArgs> > cached_roadmap
         = boost::dynamic_pointer_cast< or_lemur::RoadmapCached<ompl_lemur::LEMUR::RoadmapArgs> >(ompl_planner->_roadmap);
      if (cached_roadmap)
      {
         printf("saving cached roadmap ...\n");
         ompl_planner->_roadmap->as< or_lemur::RoadmapCached<ompl_lemur::LEMUR::RoadmapArgs> >()->save_file();
      }
      else
      {
         throw OpenRAVE::openrave_exception("asked to save roadmap cache, but non-cached roadmap used!");
      }
   }
   
   ompl_planner->as<ompl_lemur::LEMUR>()->os_alglog = 0;
   fp_alglog.close();
   
   if (params->graph == "-")
   {
      ompl_planner->as<ompl_lemur::LEMUR>()->dump_graph(std::cout);
   }
   else if (params->has_graph)
   {
      std::ofstream fp_graph;
      fp_graph.open(params->graph.c_str());
      ompl_planner->as<ompl_lemur::LEMUR>()->dump_graph(fp_graph);
      fp_graph.close();
   }
   
   if (ompl_status != ompl::base::PlannerStatus::EXACT_SOLUTION) return OpenRAVE::PS_Failed;
   
   // convert result
   // (if the planner exited with an exact empty solution, then it's done!)
   ompl::base::PathPtr path = ompl_planner->getProblemDefinition()->getSolutionPath();
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

bool or_lemur::LEMUR::GetTimes(std::ostream & sout, std::istream & sin) const
{
   sout << "checktime " << boost::chrono::duration<double>(ompl_checker->dur_checks).count();
   sout << " totaltime " << 0.0;
   sout << " n_checks " << ompl_checker->num_checks;
   sout << " e8_dur_total " <<  ompl_planner->as<ompl_lemur::LEMUR>()->getDurTotal();
   sout << " e8_dur_roadmapgen " <<  ompl_planner->as<ompl_lemur::LEMUR>()->getDurRoadmapGen();
   sout << " e8_dur_roadmapinit " <<  ompl_planner->as<ompl_lemur::LEMUR>()->getDurRoadmapInit();
   sout << " e8_dur_lazysp " <<  ompl_planner->as<ompl_lemur::LEMUR>()->getDurLazySP();
   sout << " e8_dur_search " <<  ompl_planner->as<ompl_lemur::LEMUR>()->getDurSearch();
   sout << " e8_dur_eval " <<  ompl_planner->as<ompl_lemur::LEMUR>()->getDurEval();
   sout << " e8_dur_selector_init " <<  ompl_planner->as<ompl_lemur::LEMUR>()->getDurSelectorInit();
   sout << " e8_dur_selector " <<  ompl_planner->as<ompl_lemur::LEMUR>()->getDurSelector();
   sout << " e8_dur_selector_notify " <<  ompl_planner->as<ompl_lemur::LEMUR>()->getDurSelectorNotify();
   return true;
}
