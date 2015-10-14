/* File: planner_e8roadmap.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <openrave/openrave.h>

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

#include <pr_bgl/edge_indexed_graph.h>
#include <pr_bgl/overlay_manager.h>
#include <pr_bgl/string_map.h>
#include <pr_bgl/heap_indexed.h>

#include <ompl_multiset/util.h>
#include <ompl_multiset/EffortModel.h>
#include <ompl_multiset/SimpleEffortModel.h>
#include <ompl_multiset/FnString.h>
#include <ompl_multiset/SamplerGenMonkeyPatch.h>
#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapAAGrid.h>
#include <ompl_multiset/RoadmapFromFile.h>
#include <ompl_multiset/RoadmapHalton.h>
#include <ompl_multiset/RoadmapHaltonDens.h>
#include <ompl_multiset/RoadmapRGG.h>
#include <ompl_multiset/RoadmapRGGDensConst.h>
#include <ompl_multiset/RoadmapID.h>
#include <ompl_multiset/BisectPerm.h>
#include <ompl_multiset/E8Roadmap.h>

#include "or_checker.h"
#include "planner_e8roadmap.h"


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

or_multiset::E8Roadmap::PlannerParameters::PlannerParameters():
   roadmap_id(""),
   coeff_distance(1.), coeff_checkcost(0.), coeff_batch(0.)
{
   _vXMLParameters.push_back("roadmap_id");
   _vXMLParameters.push_back("coeff_distance");
   _vXMLParameters.push_back("coeff_checkcost");
   _vXMLParameters.push_back("coeff_batch");
   _vXMLParameters.push_back("alglog");
   _vXMLParameters.push_back("graph");
}

bool
or_multiset::E8Roadmap::PlannerParameters::serialize(std::ostream& sout, int options) const
{
   if (!OpenRAVE::PlannerBase::PlannerParameters::serialize(sout))
      return false;
   sout << "<roadmap_id>" << roadmap_id << "</roadmap_id>";
   sout << "<coeff_distance>" << coeff_distance << "</coeff_distance>";
   sout << "<coeff_checkcost>" << coeff_checkcost << "</coeff_checkcost>";
   sout << "<coeff_batch>" << coeff_batch << "</coeff_batch>";
   sout << "<alglog>" << alglog << "</alglog>";
   sout << "<graph>" << graph << "</graph>";
   return !!sout;
}

OpenRAVE::BaseXMLReader::ProcessElement
or_multiset::E8Roadmap::PlannerParameters::startElement(
   const std::string & name, const OpenRAVE::AttributesList & atts)
{
   if (el_deserializing.size())
      return PE_Ignore;
   // ask base calss
   enum OpenRAVE::BaseXMLReader::ProcessElement base;
   base = OpenRAVE::PlannerBase::PlannerParameters::startElement(name,atts);
   if (base != PE_Pass) return base;
   // can we handle it?
   if (name == "roadmap_id"
      || name == "coeff_distance"
      || name == "coeff_checkcost"
      || name == "coeff_batch"
      || name == "alglog"
      || name == "graph")
   {
      el_deserializing = name;
      return PE_Support;
   }
   return PE_Pass;
}

bool
or_multiset::E8Roadmap::PlannerParameters::endElement(const std::string & name)
{
   if (!el_deserializing.size())
      return OpenRAVE::PlannerBase::PlannerParameters::endElement(name);
   if (name == el_deserializing)
   {
      if (el_deserializing == "roadmap_id")
         roadmap_id = _ss.str();
      if (el_deserializing == "coeff_distance")
         _ss >> coeff_distance;
      if (el_deserializing == "coeff_checkcost")
         _ss >> coeff_checkcost;
      if (el_deserializing == "coeff_batch")
         _ss >> coeff_batch;
      if (el_deserializing == "alglog")
         alglog = _ss.str();
      if (el_deserializing == "graph")
         graph = _ss.str();
   }
   else
      RAVELOG_WARN("closing tag doesnt match opening tag!\n");
   el_deserializing.clear();
   return false;
}

or_multiset::E8Roadmap::E8Roadmap(OpenRAVE::EnvironmentBasePtr env):
   OpenRAVE::PlannerBase(env), env(env)
{
   __description = "E8 roadmap planner";
   RegisterCommand("GetTimes",
      boost::bind(&or_multiset::E8Roadmap::GetTimes,this,_1,_2),
      "get timing information from last plan");
}

or_multiset::E8Roadmap::~E8Roadmap()
{
}

bool
or_multiset::E8Roadmap::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & inparams_ser)
{
   PlannerParametersPtr inparams(new PlannerParameters());
   inparams_ser >> *inparams;
   inparams->Validate();
   return this->InitPlan(robot, inparams);
}

bool
or_multiset::E8Roadmap::InitPlan(OpenRAVE::RobotBasePtr inrobot, OpenRAVE::PlannerBase::PlannerParametersConstPtr inparams_base)
{
   PlannerParametersConstPtr inparams = boost::dynamic_pointer_cast<PlannerParameters const>(inparams_base);
   if (!inparams)
   {
      RAVELOG_WARN("Warning, MultiSetPRM planner passed an unknown PlannerParameters type! Attempting to serialize ...\n");
      std::stringstream inparams_ser;
      inparams_ser << *inparams_base;
      return this->InitPlan(inrobot, inparams_ser);
   }
   if (!inrobot || !inparams_base)
      throw OpenRAVE::openrave_exception("robot/params objects must be passed!");
   if (!robot)
   {
      // do setup (at most once)
      params = inparams;
      robot = inrobot;
      robot_adofs = inrobot->GetActiveDOFIndices();
      
      // set up ompl space
      ompl_space.reset(new ompl::base::RealVectorStateSpace(robot_adofs.size()));
      ompl_space->as<ompl::base::RealVectorStateSpace>()->setBounds(ompl_bounds(robot));
      ompl_space->setLongestValidSegmentFraction(ompl_resolution(robot) / ompl_space->getMaximumExtent());
      ompl_space->setup();
      
      // set up si / checker
      ompl_si.reset(new ompl::base::SpaceInformation(ompl_space));
      ompl_checker.reset(new or_multiset::OrChecker(ompl_si, env, robot, robot_adofs.size()));
      ompl_si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(ompl_checker));
      ompl_si->setup();
      
      // set up planner
      sem.reset(new ompl_multiset::SimpleEffortModel(ompl_si, ompl_space->getLongestValidSegmentLength()));
      tag_cache.reset(new ompl_multiset::DummyTagCache());
      try
      {
         roadmapgen.reset(ompl_multiset::make_roadmap_gen<ompl_multiset::E8Roadmap::Roadmap>(ompl_space, inparams->roadmap_id));
      }
      catch (const std::runtime_error & ex)
      {
         throw OpenRAVE::openrave_exception("failure to create roadmap!");
      }
      ompl_planner.reset(new ompl_multiset::E8Roadmap(ompl_si, *sem, *tag_cache, roadmapgen, 1));
   }
   
   // check consistency
   if (robot != inrobot)
      throw OpenRAVE::openrave_exception("planner supports only one robot!");
   if (robot_adofs != inrobot->GetActiveDOFIndices())
      throw OpenRAVE::openrave_exception("planner supports only one robot adofs!");
   if (params->roadmap_id != inparams->roadmap_id)
      throw OpenRAVE::openrave_exception("planner supports only one roadmap_id!");
   params = inparams;
   
   // planner params
   ompl_planner->coeff_checkcost = params->coeff_checkcost;
   ompl_planner->coeff_distance = params->coeff_distance;
   ompl_planner->coeff_batch = params->coeff_batch;
   sem->has_changed_called = false; // force reeval of wlazy
   
   // problem definition
   ompl_pdef.reset(new ompl::base::ProblemDefinition(ompl_si));
   ompl_set_roots(ompl_pdef, params);   
   ompl_planner->setProblemDefinition(ompl_pdef);
   
   return true;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr
or_multiset::E8Roadmap::GetParameters() const
{
   return params;
}

OpenRAVE::PlannerStatus
or_multiset::E8Roadmap::PlanPath(OpenRAVE::TrajectoryBasePtr traj)
{
   printf("planning ...\n");
   
   ompl_checker->num_checks = 0;
   
   std::ofstream fp_alglog;
   if (params->alglog != "")
   {
      fp_alglog.open(params->alglog.c_str());
      ompl_planner->as<ompl_multiset::E8Roadmap>()->os_alglog = &fp_alglog;
   }
   
   ompl::base::PlannerStatus ompl_status;
   ompl_status = ompl_planner->solve(ompl::base::timedPlannerTerminationCondition(10.0));
   printf("planner returned: %s\n", ompl_status.asString().c_str());
   printf("planner performed %lu checks!\n", ompl_checker->num_checks);
   
   ompl_planner->as<ompl_multiset::E8Roadmap>()->os_alglog = 0;
   fp_alglog.close();
   
   if (params->graph != "")
   {
      std::ofstream fp_graph;
      fp_graph.open(params->graph.c_str());
      ompl_planner->as<ompl_multiset::E8Roadmap>()->dump_graph(fp_graph);
      fp_graph.close();
   }
   
   if (ompl_status != ompl::base::PlannerStatus::EXACT_SOLUTION) return OpenRAVE::PS_Failed;
   
   // convert result
   ompl::base::PathPtr path = this->ompl_planner->getProblemDefinition()->getSolutionPath();
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

bool or_multiset::E8Roadmap::GetTimes(std::ostream & sout, std::istream & sin) const
{
   sout << "checktime " << 0.0;
   sout << " totaltime " << 0.0;
   sout << " n_checks " << ompl_checker->num_checks;
   return true;
}
