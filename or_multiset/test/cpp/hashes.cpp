/* File: hashes.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <openrave/openrave.h>
#include <openrave-core.h>

#include <gtest/gtest.h>

#define XSTR(s) STR(s)
#define STR(s) # s

TEST(HashesTestCase, HashesTest)
{
   bool success;
   
   // initialize with local wam7
   OpenRAVE::RaveInitialize(true, OpenRAVE::Level_Info);
   OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();
   OpenRAVE::RobotBasePtr robot = env->ReadRobotXMLFile(OpenRAVE::RobotBasePtr(),
      XSTR(ORDATA) "/barrettwam/barrettwam.robot.xml");
   ASSERT_TRUE(robot);
   env->Add(robot);
   
   // set active dofs
   std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manips = robot->GetManipulators();
   robot->SetActiveDOFs(manips[0]->GetArmIndices());
   ASSERT_EQ(robot->GetActiveDOF(), 7);
   
   // construct planner
   OpenRAVE::PlannerBasePtr planner = OpenRAVE::RaveCreatePlanner(env, "E8RoadmapSelfCC");
   ASSERT_TRUE(planner);
   
   // construct planner parameters
   OpenRAVE::PlannerBase::PlannerParametersPtr params(new OpenRAVE::PlannerBase::PlannerParameters());
   params->SetRobotActiveJoints(robot);
   params->vinitialconfig.clear();
   params->vgoalconfig.clear();
   params->_sExtraParameters = "<roadmap_id>RGG(n=1000 radius=2.0 seed=1)</roadmap_id>";
   params->Validate();
   success = planner->InitPlan(robot, params);
   ASSERT_TRUE(success);
   
   // get hash
   std::stringstream ssin("GetSelfHash");
   std::stringstream ssout;
   success = planner->SendCommand(ssout,ssin);
   ASSERT_TRUE(success);
   ASSERT_EQ("8eee198040256418c9e1613b79cba6ff", ssout.str());
   
   env->Destroy();
   OpenRAVE::RaveDestroy();
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
