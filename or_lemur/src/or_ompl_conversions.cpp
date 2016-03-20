/*! \file or_ompl_conversions.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#include <vector>

#include <boost/chrono.hpp>

#include <openrave/openrave.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/ProblemDefinition.h>

#include <or_lemur/or_ompl_conversions.h>

ompl::base::RealVectorBounds or_lemur::ompl_bounds(OpenRAVE::RobotBasePtr robot)
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

double or_lemur::ompl_resolution(OpenRAVE::RobotBasePtr robot)
{
   std::vector<OpenRAVE::dReal> dof_resolutions;
   robot->GetActiveDOFResolutions(dof_resolutions);
   double resolution = HUGE_VAL;
   for (unsigned int i=0; i<dof_resolutions.size(); i++)
      resolution = dof_resolutions[i] < resolution ? dof_resolutions[i] : resolution;
   return resolution;
}

bool or_lemur::ompl_set_roots(ompl::base::ProblemDefinitionPtr ompl_pdef,
   OpenRAVE::PlannerBase::PlannerParametersConstPtr params)
{
   ompl::base::SpaceInformationPtr space_si = ompl_pdef->getSpaceInformation();
   ompl::base::StateSpacePtr space = space_si->getStateSpace();
   unsigned int dim = space->getDimension();
   
   // add start states
   ompl_pdef->clearStartStates();
   if (params->vinitialconfig.size() % dim != 0)
   {
      RAVELOG_ERROR("Vector of initial states is not the right size.\n");
      return false;
   }
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
   {
      RAVELOG_ERROR("Vector of goal states is not the right size.\n");
      return false;
   }
   unsigned int num_goals = params->vgoalconfig.size() / dim;
   for (unsigned int igoal=0; igoal<num_goals; igoal++)
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(space);
      for (unsigned int j=0; j<dim; j++)
         s_goal->values[j] = params->vgoalconfig[igoal*dim + j];
      gs->addState(s_goal);
   }
   ompl_pdef->setGoal(ompl::base::GoalPtr(gs));
   return true;
}
