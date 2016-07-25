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

#include <or_lemur/config.h>
#include <or_lemur/SemiTorusStateSpace.h>
#include <or_lemur/or_ompl_conversions.h>

double or_lemur::ompl_resolution(
   const OpenRAVE::RobotBasePtr & robot,
   const std::vector<int> & dofs)
{
   std::vector<OpenRAVE::dReal> dof_resolutions;
   robot->GetDOFResolutions(dof_resolutions, dofs);
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

bool or_lemur::create_space(
   const OpenRAVE::RobotBasePtr & robot,
   const std::vector<int> & dofs,
   bool do_checker,
   bool do_baked,
   ompl::base::SpaceInformationPtr & out_space_info)
{
   // get bounds / circularity of dofs
   ompl::base::RealVectorBounds bounds(dofs.size());
   std::vector<bool> is_circular(dofs.size());
   bool any_is_circular = false;
   for (std::size_t idof=0; idof<dofs.size(); idof++)
   {
      OpenRAVE::RobotBase::JointPtr joint = robot->GetJointFromDOFIndex(dofs[idof]);
      int iaxis = dofs[idof] - joint->GetDOFIndex();
      // circularity
      is_circular[idof] = joint->IsCircular(iaxis);
      if (is_circular[idof])
         any_is_circular = true;
      // bounds
      std::pair<OpenRAVE::dReal, OpenRAVE::dReal> limits = joint->GetLimit(iaxis);
      if (is_circular[idof])
      {
         double diam = limits.second - limits.first;
         if (fabs(diam - 2.0*M_PI) > std::numeric_limits<double>::epsilon() * 2.0)
         {
            RAVELOG_WARN("Robot DOF [%lu] is circular, but has limits [%f,%f]!\n",
               idof, limits.first,limits.second);
            RAVELOG_WARN("Ignoring limits and using [-PI,PI] instead ...\n");
            limits.first = -M_PI;
            limits.second = M_PI;
         }
      }
      bounds.setLow(idof, limits.first);
      bounds.setHigh(idof, limits.second);
   }
   
   // construct space
   ompl::base::StateSpacePtr space;
   if (any_is_circular)
   {
      space.reset(new or_lemur::SemiTorusStateSpace(dofs.size()));
      space->as<or_lemur::SemiTorusStateSpace>()->setBounds(bounds);
      space->as<or_lemur::SemiTorusStateSpace>()->setIsWrapping(is_circular);
   }
   else
   {
      space.reset(new ompl::base::RealVectorStateSpace(dofs.size()));
      space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
   }
   space->setLongestValidSegmentFraction(ompl_resolution(robot,dofs) / space->getMaximumExtent());
   space->setup();
   
   // set up si
   out_space_info.reset(new ompl::base::SpaceInformation(space));
   
   // set up checker
   if (do_checker)
   {
      ompl::base::StateValidityCheckerPtr checker(
         new or_lemur::OrChecker(out_space_info,
         robot->GetEnv(), robot, dofs.size(), do_baked));
      out_space_info->setStateValidityChecker(checker);
      out_space_info->setup();
   }
   
   return true;
}
