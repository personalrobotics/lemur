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

double checkcost_measured;

// robot (circle)
bool isvalid_r(const ompl::base::State * s)
{
   checkcost_measured += 10.0;
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   if (pow(q[0]-0.5,2) + pow(q[1]-0.5,2) < 0.3*0.3)
      return false;
   else
      return true;
}

// robot padded (bounding box)
bool isvalid_p(const ompl::base::State * s)
{
   checkcost_measured += 1.0;
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   if (0.2 < q[0] && q[0] < 0.8 && 0.2 < q[1] && q[1] < 0.8)
      return false;
   else
      return true;
}

// horizontal bar
bool isvalid_a(const ompl::base::State * s)
{
   checkcost_measured += 1.0;
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   if (0.1 < q[0] && q[0] < 0.9 && 0.4 < q[1] && q[1] < 0.6)
      return false;
   else
      return true;
}

// vertical bar
bool isvalid_b(const ompl::base::State * s)
{
   checkcost_measured += 1.0;
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   if (0.4 < q[0] && q[0] < 0.6 && 0.1 < q[1] && q[1] < 0.9)
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

bool isvalid_rb(const ompl::base::State * s)
{
   if (!isvalid_r(s)) return false;
   if (!isvalid_b(s)) return false;
   return true;
}


bool isvalid_ra_broadnarrow(const ompl::base::State * s)
{
   if (!isvalid_p(s) && !isvalid_r(s)) return false;
   if (!isvalid_a(s)) return false;
   return true;
}

int main(int argc, char * argv[])
{
   ompl::base::StateSpacePtr space;
   ompl::base::SpaceInformationPtr si_r;
   ompl::base::SpaceInformationPtr si_p;
   ompl::base::SpaceInformationPtr si_a;
   ompl::base::SpaceInformationPtr si_b;
   ompl::base::SpaceInformationPtr si_ra;
   ompl::base::SpaceInformationPtr si_ra_bn;
   ompl::base::SpaceInformationPtr si_rb;
   ompl::base::ProblemDefinitionPtr pdef_ra;
   ompl::base::ProblemDefinitionPtr pdef_ra_bn;
   ompl::base::ProblemDefinitionPtr pdef_rb;
   ompl::base::PlannerPtr prm_planner;
   
   if (argc != 2)
   {
      printf("pass seed as argument!\n");
      abort();
   }
   
   ompl::RNG::setSeed(atoi(argv[1]));
   
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
   /* set space resolution */
   space->setLongestValidSegmentFraction(0.01 / space->getMaximumExtent());
   
   /* create planner */
   checkmask::GraphPlanner * p = checkmask::GraphPlanner::create(space);
   p->set_batchsize(100);
   p->set_radius(0.2);
   
   /* space information containers for each cfree */
   si_r.reset(new ompl::base::SpaceInformation(space));
   si_r->setStateValidityChecker(&isvalid_r);
   p->add_cfree(si_r, "r", 10.0);
   si_p.reset(new ompl::base::SpaceInformation(space));
   si_p->setStateValidityChecker(&isvalid_p);
   p->add_cfree(si_p, "p", 1.0);
   si_a.reset(new ompl::base::SpaceInformation(space));
   si_a->setStateValidityChecker(&isvalid_a);
   p->add_cfree(si_a, "a", 1.0);
   si_b.reset(new ompl::base::SpaceInformation(space));
   si_b->setStateValidityChecker(&isvalid_b);
   p->add_cfree(si_b, "b", 1.0);
   si_ra.reset(new ompl::base::SpaceInformation(space));
   si_ra->setStateValidityChecker(&isvalid_ra);
   p->add_cfree(si_ra, "ra", 11.0);
   si_rb.reset(new ompl::base::SpaceInformation(space));
   si_rb->setStateValidityChecker(&isvalid_rb);
   p->add_cfree(si_rb, "rb", 11.0);
   si_ra_bn.reset(new ompl::base::SpaceInformation(space));
   si_ra_bn->setStateValidityChecker(&isvalid_ra_broadnarrow);
   p->add_cfree(si_ra_bn, "ra_bn", 11.0);
   
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
   
   pdef_rb.reset(new ompl::base::ProblemDefinition(si_rb));
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(space);
      s_start->values[0] = 0.1;
      s_start->values[1] = 0.1;
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(space);
      s_goal->values[0] = 0.9;
      s_goal->values[1] = 0.9;
      pdef_rb->setStartAndGoalStates(s_start, s_goal);
   }
   
   pdef_ra_bn.reset(new ompl::base::ProblemDefinition(si_ra_bn));
   {
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_start(space);
      s_start->values[0] = 0.1;
      s_start->values[1] = 0.1;
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s_goal(space);
      s_goal->values[0] = 0.9;
      s_goal->values[1] = 0.9;
      pdef_ra_bn->setStartAndGoalStates(s_start, s_goal);
   }
   
   /* define set relations */
   //p->add_inclusion(si_r, si_p);
   //p->add_intersection(si_r, si_a, si_ra);
   //p->add_intersection(si_r, si_b, si_rb);
   
   /* ask the checkmask planner to solve our problem */

#if 1
   checkcost_measured = 0.0;
   p->setProblemDefinition(pdef_ra_bn);
   ompl::base::PlannerStatus status = p->solve(ompl::base::timedPlannerTerminationCondition(600.0));
   if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
   {
      printf("no solution found after 600 seconds!\n");
      abort();
   }
   printf("solved, cost: %f\n", checkcost_measured);
#endif
   
#if 0
   checkcost_measured = 0.0;
   p->setProblemDefinition(pdef_ra);
   p->solve(ompl::base::timedPlannerTerminationCondition(1.0));
   double cost_ra = checkcost_measured;

   checkcost_measured = 0.0;
   p->setProblemDefinition(pdef_rb);
   p->solve(ompl::base::timedPlannerTerminationCondition(1.0));
   double cost_rb = checkcost_measured;
   printf("checkcost ra: %f rb: %f\n", cost_ra, cost_rb);
#endif
   
   delete p;
   
   return 0;
}
