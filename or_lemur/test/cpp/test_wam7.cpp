/*! \file test_wam7.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#include <openrave/openrave.h>
#include <openrave-core.h>

#include <gtest/gtest.h>

OpenRAVE::dReal q_start[7] = {-2.0, -0.5, -0.2, -0.5, 0.0, 0.0, 0.0};
OpenRAVE::dReal q_goal[7] = { 2.0,  0.5,  0.2,  0.5, 0.0, 0.0, 0.0};

OpenRAVE::dReal traj_expected[6][7] = {
   { -2.00000, -0.50000, -0.20000, -0.50000,  0.00000,  0.00000,  0.00000 },
   { -1.38058, -0.26242, -0.88124,  0.01679, -0.04491, -0.39824,  0.55176 },
   { -0.64427, -0.35982, -0.09207,  1.07010,  0.25800, -0.95592, -0.21814 },
   { -0.03068, -0.10010, -0.30251,  1.83249, -0.29275, -0.30386, -0.07149 },
   {  1.11981,  0.04238, -0.52874,  0.80426, -0.22390, -0.07650,  0.53342 },
   {  2.00000,  0.50000,  0.20000,  0.50000,  0.00000,  0.00000,  0.00000 }
};

TEST(Wam7TestCase, Wam7Test)
{
   bool success;
   std::stringstream ssin;
   std::stringstream ssout;
   
   // initialize with wam7
   OpenRAVE::RaveInitialize(true, OpenRAVE::Level_Info);
   OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();
   OpenRAVE::RobotBasePtr robot = env->ReadRobotXMLFile(OpenRAVE::RobotBasePtr(), "robots/barrettwam.robot.xml");
   ASSERT_TRUE(robot);
   env->Add(robot);
   std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manips = robot->GetManipulators();
   robot->SetActiveDOFs(manips[0]->GetArmIndices());
   ASSERT_EQ(robot->GetActiveDOF(), 7);
   
   // construct planner
   OpenRAVE::PlannerBasePtr planner = OpenRAVE::RaveCreatePlanner(env, "LEMUR");
   ASSERT_TRUE(planner);
   
   // construct planner parameters
   OpenRAVE::PlannerBase::PlannerParametersPtr params(new OpenRAVE::PlannerBase::PlannerParameters());
   params->SetRobotActiveJoints(robot);
   params->SetConfigurationSpecification(env, robot->GetActiveConfigurationSpecification());
   params->vinitialconfig = std::vector<OpenRAVE::dReal>(q_start, q_start+7);
   params->vgoalconfig = std::vector<OpenRAVE::dReal>(q_goal, q_goal+7);
   params->_sExtraParameters =
      "<roadmap_type>Halton</roadmap_type>"
      "<roadmap>"
      "<num>1000</num>"
      "<radius>2.0</radius>"
      "</roadmap>";
   
   params->Validate();
   success = planner->InitPlan(robot, params);
   ASSERT_TRUE(success);
   
   // plan
   OpenRAVE::TrajectoryBasePtr traj = OpenRAVE::RaveCreateTrajectory(env);
   ASSERT_TRUE(traj);
   OpenRAVE::PlannerStatus status = planner->PlanPath(traj);
   ASSERT_EQ(OpenRAVE::PS_HasSolution, status);
   
   // validate trajectory
   const OpenRAVE::ConfigurationSpecification & tracj_cspec = traj->GetConfigurationSpecification();
   unsigned int gidx;
   for (gidx=0; gidx<tracj_cspec._vgroups.size(); gidx++)
      if (tracj_cspec._vgroups[gidx].name.substr(0,13) == "joint_values ")
         break;
   ASSERT_LT(gidx, tracj_cspec._vgroups.size());
   const OpenRAVE::ConfigurationSpecification::Group & group = tracj_cspec._vgroups[gidx];
   ASSERT_EQ(7, group.dof);
   
   // validate trajectory
   ASSERT_EQ(6, traj->GetNumWaypoints());
   for (unsigned int iwp=0; iwp<6; iwp++)
   {
      std::vector<OpenRAVE::dReal> wp;
      traj->GetWaypoint(iwp, wp);
      for (unsigned int i=0; i<7; i++)
         ASSERT_NEAR(traj_expected[iwp][i], wp[i], 1.0e-5);
   }
   
   env->Destroy();
   OpenRAVE::RaveDestroy();
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
