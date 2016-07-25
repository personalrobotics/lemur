/*! \file test_module_family.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#include <openrave/openrave.h>
#include <openrave-core.h>

#include <or_lemur/module_family.h>

#include <gtest/gtest.h>

struct testcheck
{
   OpenRAVE::dReal config[7];
   bool result;
};

struct testcheck testchecks[] =
{
   {{-2.0, -0.5, -0.2, -0.5, 0.0, 0.0, 0.0}, true},
   {{ 2.0,  0.5,  0.2,  0.5, 0.0, 0.0, 0.0}, true},
   {{ 0.0,  0.0,  0.0,  3.0, 0.0, 0.0, 0.0}, false},
};

TEST(FamilyModuleTestCase, FamilyModuleTest)
{
   //bool success;
   std::stringstream ssin;
   std::stringstream ssout;
   
   // initialize with wam7
   OpenRAVE::RaveInitialize(true, OpenRAVE::Level_Info);
   OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();
   OpenRAVE::RobotBasePtr robot = env->ReadRobotXMLFile(OpenRAVE::RobotBasePtr(), "robots/barrettwam.robot.xml");
   ASSERT_TRUE(robot.get());
   env->Add(robot);
   std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manips = robot->GetManipulators();
   robot->SetActiveDOFs(manips[0]->GetArmIndices());
   ASSERT_EQ(robot->GetActiveDOF(), 7);
   
   // test configurations the old fashioned way
   for (unsigned int ui=0; ui<sizeof(testchecks)/sizeof(testchecks[0]); ui++)
   {
      std::vector<OpenRAVE::dReal> values(testchecks[ui].config, testchecks[ui].config+7);
      robot->SetActiveDOFValues(values);
      bool collided = env->CheckCollision(robot) || robot->CheckSelfCollision();
      bool valid = !collided;
      printf("testcheck [%u] is %s!\n", ui, valid ? "valid" : "invalid");
      ASSERT_EQ(testchecks[ui].result, valid);
   }
   
   // construct family
   OpenRAVE::ModuleBasePtr mod = OpenRAVE::RaveCreateModule(env, "Family");
   ASSERT_TRUE(mod.get());
   env->Add(mod, false, "--robot-name=BarrettWAM");
   
   // do cast
   boost::shared_ptr<or_lemur::FamilyModule> modfamily
      = boost::static_pointer_cast<or_lemur::FamilyModule>(mod);
   
   // get current set
   or_lemur::FamilyModule::SetPtr set = modfamily->GetCurrentSet();
   or_lemur::FamilyModule::Indicator indicator;
   
   // get indicator
   {
      or_lemur::FamilyModule::Family fam;
      fam.sets.insert(set);
      std::map< or_lemur::FamilyModule::SetPtr, std::pair<double,or_lemur::FamilyModule::Indicator> >
         indicators = modfamily->GetIndicators(fam);
      indicator = indicators[set].second;
   }
   
   // test configurations
   for (unsigned int ui=0; ui<sizeof(testchecks)/sizeof(testchecks[0]); ui++)
   {
      std::vector<OpenRAVE::dReal> values(testchecks[ui].config, testchecks[ui].config+7);
      bool valid = indicator(values);
      printf("testcheck [%u] is %s!\n", ui, valid ? "valid" : "invalid");
      ASSERT_EQ(testchecks[ui].result, valid);
   }
   
   // get current family (should have no set, nothing is bound so far!)
   or_lemur::FamilyModule::Family family = modfamily->GetCurrentFamily();
   ASSERT_EQ(0, family.sets.size());
   
   // ok, bind to the current set
   
   
   
   //ASSERT_EQ(set, *family.begin());
   
   env->Destroy();
   OpenRAVE::RaveDestroy();
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
