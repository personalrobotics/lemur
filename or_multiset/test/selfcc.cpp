/* File: selfcc.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <openrave/openrave.h>
#include <openrave-core.h>

#include <gtest/gtest.h>

OpenRAVE::Transform mk_ortx(
   double x, double y, double z,
   double qx, double qy, double qz, double qw)
{
   return OpenRAVE::Transform(
      OpenRAVE::Vector(qw,qx,qy,qz),
      OpenRAVE::Vector(x,y,z,0.));
}

TEST(SelfCCTestCase, SimpleTest)
{
   bool success;
   // initialize with wam7
   OpenRAVE::RaveInitialize(true, OpenRAVE::Level_Info);
   OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();
   OpenRAVE::RobotBasePtr robot = env->ReadRobotXMLFile(OpenRAVE::RobotBasePtr(), "robots/barrettwam.robot.xml");
   ASSERT_TRUE(robot);
   env->Add(robot);
   std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manips = robot->GetManipulators();
   robot->SetActiveDOFs(manips[0]->GetArmIndices());
   ASSERT_EQ(robot->GetActiveDOF(), 7);
   
   robot->GetLink("wam5")->Enable(false);
   
   // add an object
   OpenRAVE::KinBodyPtr mug3 = env->ReadKinBodyURI("models/objects/mug3.iv");
   env->Add(mug3);
   mug3->SetTransform(mk_ortx(1,0,0, 0,0,0,1));
   // construct planner parameters
   OpenRAVE::PlannerBase::PlannerParametersPtr params(new OpenRAVE::PlannerBase::PlannerParameters());
   params->SetRobotActiveJoints(robot);
   params->SetConfigurationSpecification(env, robot->GetActiveConfigurationSpecification());
   // trivial planning query
   params->vinitialconfig.clear();
   params->vgoalconfig.clear();
   params->_sExtraParameters = "<roadmap_id>RGGDensConst(n_perbatch=1000 radius=1.0 seed=1)</roadmap_id>";
   params->Validate();
   // call planner
   OpenRAVE::PlannerBasePtr planner = OpenRAVE::RaveCreatePlanner(env, "E8RoadmapSelfCC");
   ASSERT_TRUE(planner);
   printf("calling init ...\n");
   success = planner->InitPlan(robot, params);
   printf("init called!\n");
   ASSERT_TRUE(success);
   
   std::stringstream ssin;
   std::stringstream ssout;
   ssin << "CacheCalculateSave";
   success = planner->SendCommand(ssout,ssin);
   ASSERT_TRUE(success);
   
#if 0
   OpenRAVE::TrajectoryBasePtr traj = OpenRAVE::RaveCreateTrajectory(env);
   ASSERT_TRUE(traj);
   OpenRAVE::PlannerStatus status = planner->PlanPath(traj);
   ASSERT_EQ(status, OpenRAVE::PS_HasSolution);
   // validate trajectory
   const OpenRAVE::ConfigurationSpecification & tracj_cspec = traj->GetConfigurationSpecification();
   unsigned int gidx;
   for (gidx=0; gidx<tracj_cspec._vgroups.size(); gidx++)
      if (tracj_cspec._vgroups[gidx].name.substr(0,13) == "joint_values ")
         break;
   ASSERT_LT(gidx, tracj_cspec._vgroups.size());
   const OpenRAVE::ConfigurationSpecification::Group & group = tracj_cspec._vgroups[gidx];
   ASSERT_EQ(group.dof, 7);
   // validate endpoints
   ASSERT_GE(traj->GetNumWaypoints(), 2);
   std::vector<OpenRAVE::dReal> traj_start;
   std::vector<OpenRAVE::dReal> traj_end;
   traj->GetWaypoint(0, traj_start);
   traj->GetWaypoint(traj->GetNumWaypoints()-1, traj_end);
   for (unsigned int i=0; i<7; i++)
   {
      ASSERT_DOUBLE_EQ(params->vinitialconfig[i], traj_start[group.offset+i]);
      ASSERT_DOUBLE_EQ(params->vgoalconfig[i], traj_end[group.offset+i]);
   }
#endif
   
   env->Destroy();
   OpenRAVE::RaveDestroy();
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
