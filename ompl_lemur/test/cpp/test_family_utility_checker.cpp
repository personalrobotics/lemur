/*! \file test_family_utility_checker.cpp
 * \author Chris Dellin <cdellin@gmail.com>
 * \copyright 2015 Carnegie Mellon University
 * \copyright License: BSD
 */

#include <set>
#include <boost/graph/adjacency_list.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <pr_bgl/heap_indexed.h>
#include <ompl_lemur/config.h>
#include <ompl_lemur/logic_engine.h>
#include <ompl_lemur/UtilityChecker.h>
#include <ompl_lemur/Family.h>
#include <ompl_lemur/FamilyUtilityChecker.h>

#include <gtest/gtest.h>

// (  ( s1[c=1] )  s2[c=2]  )
//                 ^ target
TEST(FamilyUtilityCheckerCase, InclusionTest)
{
   // note that we don't set up space or si's for now
   
   ompl::base::StateSpacePtr space(
      new ompl::base::RealVectorStateSpace(1));
   ompl::base::SpaceInformationPtr si(
      new ompl::base::SpaceInformation(space));
   
   ompl_lemur::Family family;
   family.sets.insert("si1");
   family.sets.insert("si2");
   family.add_inclusion("si1","si2");
   
   ompl_lemur::FamilyUtilityChecker family_checker(si, family);
   std::map<std::string, ompl_lemur::FamilyUtilityChecker::SetChecker> set_checkers;
   set_checkers["si1"] = std::make_pair(1.0, ompl::base::StateValidityCheckerPtr());
   set_checkers["si2"] = std::make_pair(2.0, ompl::base::StateValidityCheckerPtr());
   family_checker.start_checking("si2", set_checkers);
   
   ASSERT_TRUE(family_checker.getPartialEvalCost(0,0) == 1.0);
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
