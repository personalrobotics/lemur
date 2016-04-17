/*! \file planner_cctimer.cpp
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

#include <or_lemur/or_ompl_conversions.h>
#include <or_lemur/planner_cctimer.h>


or_lemur::CCTimer::CCTimer(OpenRAVE::EnvironmentBasePtr env):
   OpenRAVE::PlannerBase(env), env(env)
{
   __description = "LEMUR collision-check timer planner";
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
   bool success = or_lemur::create_space(robot, robot_adofs, true, false, ompl_si);
   if (!success)
      return false;
   
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
   
   ompl_checker->start();
   
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
   
   ompl_checker->stop();
   
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
   sout << " dur_total " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurTotal();
   sout << " dur_roadmapgen " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurRoadmapGen();
   sout << " dur_roadmapinit " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurRoadmapInit();
   sout << " dur_lazysp " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurLazySP();
   sout << " dur_search " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurSearch();
   sout << " dur_eval " <<  ompl_planner->as<ompl_lemur::CCTimer>()->getDurEval();
#endif
   return true;
}
