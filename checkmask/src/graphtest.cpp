#include <cstdio>
#include <string>
#include <vector>
#include <boost/function.hpp>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <checkmask/graph.h>

// robot (circle)
bool isvalid_r(const ompl::base::State * s)
{
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   if (pow(q[0]-0.5,2) + pow(q[1]-0.5,2) < 0.3*0.3)
      return false;
   else
      return true;
}

// horizontal bar
bool isvalid_a(const ompl::base::State * s)
{
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   if (0.1 < q[0] && q[0] < 0.9 && 0.4 < q[1] && q[1] < 0.6)
      return false;
   else
      return true;
}

bool isvalid_ra(const ompl::base::State * s)
{
   if (!isvalid_r(s)) return false;
   if (!isvalid_a(s)) return false;
   return true;
}

int main()
{
   ompl::base::StateSpacePtr space;
   ompl::base::SpaceInformationPtr si_ra;
   ompl::base::ProblemDefinitionPtr pdef_ra;
   ompl::base::GoalPtr prm_goals;
   ompl::base::PlannerPtr prm_planner;
   
   printf("starting test ...\n");
   
   /* state space (2D circle+cross) */
   space.reset(new ompl::base::RealVectorStateSpace(2));
   /* state space set bounds */
   {
      ompl::base::RealVectorBounds bounds(2);
      bounds.setLow(0, 0.0); bounds.setHigh(0, 1.0);
      bounds.setLow(1, 0.0); bounds.setHigh(1, 1.0);
      space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
   }
   
   /* create planner (with dummy si based on our space) */
   checkmask::GraphPlanner * p = checkmask::GraphPlanner::create(space);
   
   /* space information container
    * for now, just for RA problem */
   si_ra.reset(new ompl::base::SpaceInformation(space));
   si_ra->setStateValidityCheckingResolution(0.01);
   si_ra->setStateValidityChecker(&isvalid_ra);
   si_ra->setup();
   
   /* create a problem definition from this si_ra container
    * (with start and goal configs) */
   pdef_ra.reset(new ompl::base::ProblemDefinition(si_ra));
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(space);
      s_start->values[0] = 0.1;
      s_start->values[1] = 0.1;
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(space);
      s_goal->values[0] = 0.9;
      s_goal->values[1] = 0.9;
      pdef_ra->setStartAndGoalStates(s_start, s_goal);
   }
   
   /* ask the checkmask planner to solve our problem */
   p->setProblemDefinition(pdef_ra);
   printf("trying once ...\n");
   p->solve(ompl::base::timedPlannerTerminationCondition(1.0));
   //printf("trying again ...\n");
   //p->solve(ompl::base::timedPlannerTerminationCondition(1.0));
   
   
   
   
   
   
   delete p;
   
   return 0;
}
