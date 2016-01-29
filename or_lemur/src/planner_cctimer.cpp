/* File: planner_cctimer.cpp
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

#include <or_lemur/or_checker.h>
#include <or_lemur/planner_cctimer.h>


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

} // anonymous namespace


or_lemur::CCTimer::CCTimer(OpenRAVE::EnvironmentBasePtr env):
   OpenRAVE::PlannerBase(env), env(env)
{
   __description = "E8 roadmap planner";
   RegisterCommand("GetTimes",
      boost::bind(&or_lemur::CCTimer::GetTimes,this,_1,_2),
      "get timing information from last plan");
}

or_lemur::CCTimer::~CCTimer()
{
}

bool
or_lemur::CCTimer::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream & inparams_ser)
{
   OpenRAVE::PlannerBase::PlannerParametersPtr inparams(new OpenRAVE::PlannerBase::PlannerParameters);
   inparams_ser >> *inparams;
   inparams->Validate();
   return this->InitPlan(robot, inparams);
}

bool
or_lemur::CCTimer::InitPlan(OpenRAVE::RobotBasePtr inrobot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params)
{
   robot = inrobot;
   const std::vector<int> robot_adofs = inrobot->GetActiveDOFIndices();
   
   // set up ompl space
   ompl_space.reset(new ompl::base::RealVectorStateSpace(robot_adofs.size()));
   ompl_space->as<ompl::base::RealVectorStateSpace>()->setBounds(ompl_bounds(robot));
   ompl_space->setLongestValidSegmentFraction(ompl_resolution(robot) / ompl_space->getMaximumExtent());
   ompl_space->setup();
   
   // set up si / checker
   ompl_si.reset(new ompl::base::SpaceInformation(ompl_space));
   ompl_checker.reset(new or_lemur::OrChecker(ompl_si, env, robot, robot_adofs.size()));
   ompl_si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(ompl_checker));
   ompl_si->setup();
   
   return true;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr
or_lemur::CCTimer::GetParameters() const
{
   return OpenRAVE::PlannerBase::PlannerParametersConstPtr();
}

OpenRAVE::PlannerStatus
or_lemur::CCTimer::PlanPath(OpenRAVE::TrajectoryBasePtr traj)
{
   printf("doing collision checks ...\n");
   
   ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state(ompl_space);
   ompl::base::StateSamplerPtr sampler(ompl_space->allocStateSampler());
   
   int num_valids = 0;
   int num_invalids = 0;
   boost::chrono::high_resolution_clock::duration dur_valids;
   boost::chrono::high_resolution_clock::duration dur_invalids;
   
   boost::chrono::high_resolution_clock::time_point time_begin;
   boost::chrono::high_resolution_clock::time_point time_end;
   
   for (unsigned int count=0; count<10000; count++)
   {
      sampler->sampleUniform(state.get());
      
      time_begin = boost::chrono::high_resolution_clock::now();
      bool isvalid = ompl_si->isValid(state.get());
      time_end = boost::chrono::high_resolution_clock::now();
      
      if (isvalid)
      {
         num_valids++;
         dur_valids += time_end - time_begin;
      }
      else
      {
         num_invalids++;
         dur_invalids += time_end - time_begin;
      }
   }
   
   printf("num valids: %d, average valid check time: %f s\n",
      num_valids, boost::chrono::duration<double>(dur_valids).count()/num_valids);
   printf("num invalids: %d, average invalid check time: %f s\n",
      num_invalids, boost::chrono::duration<double>(dur_invalids).count()/num_invalids);
   
   return OpenRAVE::PS_HasSolution;
}

bool or_lemur::CCTimer::GetTimes(std::ostream & sout, std::istream & sin) const
{
#if 0
   sout << "checktime " << boost::chrono::duration<double>(ompl_checker->dur_checks).count();
   sout << " totaltime " << 0.0;
   sout << " n_checks " << ompl_checker->num_checks;
   sout << " e8_dur_total " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurTotal();
   sout << " e8_dur_roadmapgen " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurRoadmapGen();
   sout << " e8_dur_roadmapinit " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurRoadmapInit();
   sout << " e8_dur_lazysp " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurLazySP();
   sout << " e8_dur_search " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurSearch();
   sout << " e8_dur_eval " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurEval();
#endif
   return true;
}
